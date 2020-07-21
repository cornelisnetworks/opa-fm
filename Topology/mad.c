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

// mad queries to PMA and DMA


#include "topology.h"
#include "topology_internal.h"
// TBD - fix conflict with ib_utils_openib.h
#define DBGPRINT(format, args...) if (g_verbose_file) { fprintf(g_verbose_file, format, ##args); }
#include <limits.h>
#include <opamgt_sa_priv.h>

// umadt timeouts for DMA and PMA operations
#define SEND_WAIT_TIME (100)	// 100 milliseconds for sends
#define FLUSH_WAIT_TIME (100)	// 100 milliseconds for sends/recv during flush
#define RESP_WAIT_TIME (100)	// 100 milliseconds for response recv

#if defined(IB_STACK_IBACCESS) || defined(CAL_IBACCESS)
static IB_HANDLE g_umadtHandle = NULL;	/* umadt handle for DM interface queries */
#endif

static uint64			g_transId		= 1234;	// transaction id
static FILE 			*g_verbose_file = NULL;	// file for verbose output
static uint64 			g_mkey = 0;				// m_Key for SMA operations
static int 				g_smaRetries = 8; 		// number of request/response attempts

FSTATUS send_recv_mad(MADT_HANDLE umadtHandle, IB_PATH_RECORD *pathp,
                             uint32 qpn, uint32 qkey, MAD* mad,
                             int timeout, int retries);

void setTopologyMadVerboseFile(FILE* verbose_file) {
	g_verbose_file = verbose_file;
}

void setTopologyMadRetryCount(int retries) {
	g_smaRetries = retries;
}

#ifdef IB_DEBUG
void DumpMad(void *addr)
{
	uint32 i;
	for (i=0; i<256; i++)
	{
		if ((i & 15) == 0) {
			if (i != 0) {
				DBGPRINT("\n");
			}
			DBGPRINT("%4.4x:", i);
		}
		DBGPRINT(" %2.2x", (uint32)((uint8*)addr)[i]);
	}
	if (i != 0) {
		DBGPRINT("\n");
	}
}

void StlDumpMad(void *addr)
{
	uint32 i;
	for (i=0; i<2048; i++)
	{
		if ((i & 15) == 0) {
			if (i != 0) {
				DBGPRINT("\n");
			}
			DBGPRINT("%4.4x:", i);
		}
		DBGPRINT(" %2.2x", (uint32)((uint8*)addr)[i]);
	}
	if (i != 0) {
		DBGPRINT("\n");
	}
}
#endif /* IB_DEBUG */

#if defined(IB_STACK_IBACCESS) || defined(CAL_IBACCESS)
static FSTATUS registerUmadt(EUI64 portGuid, MADT_HANDLE *handle)
{
	RegisterClassStruct Reg;
	FSTATUS fstatus;

	DBGPRINT("registerUmadt: Port 0x%016"PRIx64"\n", portGuid);

	Reg.PortGuid = portGuid;
	Reg.ClassId = 0;	/* only client, don't need to specify */
	Reg.ClassVersion = 0; /* only client, don't need to specify */
	Reg.isResponder = FALSE;
	Reg.isTrapProcessor = FALSE;
	Reg.isReportProcessor = FALSE;
	Reg.SendQueueSize = 5;	// TBD
	Reg.RecvQueueSize = 5;	// TBD
	Reg.NotifySendCompletion = FALSE;

	DBGPRINT("register uMadt\n");
	fstatus = iba_umadt_register(&Reg, handle);
	if (fstatus != FSUCCESS) {
		fprintf(stderr, "%s: Failed to register Port 0x%016"PRIx64" with uMadt: (fstatus=0x%x): %s\n",
						g_Top_cmdname, portGuid, fstatus, FSTATUS_MSG(fstatus));
	}
	return fstatus;
}

static FSTATUS send_mad(MADT_HANDLE umadtHandle, IB_PATH_RECORD *pathp,
				uint32 qpn, uint32 qkey, MAD* mad)
{
	FSTATUS status;
	MadtStruct *Mad;
	MadAddrStruct Addr;
	uint32 count = 1;
	MadWorkCompletion *Wc;

	DBGPRINT("getting buffer to send Mad\n");
	status = iba_umadt_get_sendmad(umadtHandle, &count, &Mad);
	if (status != FSUCCESS) {
		return status;
	}

	DBGPRINT("sending Mad\n");
	Mad->Context = 0;	// TBD - for async in future
	MemoryClear(&Mad->Grh, sizeof(Mad->Grh));	/* only in recv */
	MemoryCopy(&Mad->IBMad, mad, sizeof(*mad));
	GetGsiAddrFromPath(pathp, qpn, qkey, &Addr);

	status = iba_umadt_post_send(umadtHandle, Mad, &Addr);
	if (status != FSUCCESS) {
		fprintf(stderr, "%s: Failed to send mad: (status=0x%x): %s\n",
						g_Top_cmdname, status, FSTATUS_MSG(status));
		(void)iba_umadt_release_sendmad(umadtHandle, Mad);
		return status;
	}
	status = iba_umadt_wait_any_compl(umadtHandle, SEND_COMPLETION, SEND_WAIT_TIME);
	if (status == FSUCCESS) {
		status = iba_umadt_poll_send_compl(umadtHandle, &Mad, &Wc);
		if (status == FSUCCESS) {
			DBGPRINT("MAD Sent\n");
			(void)iba_umadt_release_sendmad(umadtHandle, Mad);
		}
	}
	return status;
}

static FSTATUS recv_mad(MADT_HANDLE umadtHandle, MAD* mad, int timeout)
{
	FSTATUS status;
	MadtStruct *Mad;
	MadWorkCompletion *Wc;

	status = iba_umadt_wait_any_compl(umadtHandle, RECV_COMPLETION, timeout);
	if (status == FTIMEOUT) {
		DBGPRINT("Timeout waiting for MAD Recv. timeout= %d ms\n", timeout);
		return status;
	}
	// cleanup any send completions while here
	do {
		status = iba_umadt_poll_send_compl(umadtHandle, &Mad, &Wc);
		if (status == FSUCCESS) {
			(void)iba_umadt_release_sendmad(umadtHandle, Mad);
		}
	} while (status == FSUCCESS);
	// get next recv from the Q
	status = iba_umadt_poll_recv_compl(umadtHandle, &Mad, &Wc);
	if (status != FSUCCESS) {
		DBGPRINT("Poll Recv failed: %s\n", iba_fstatus_msg(status));
		return status;
	}
	MemoryCopy(mad, &Mad->IBMad, sizeof(*mad));	// caller must byte swap
	// for now ignore Work completion, only handling responses
	DBGPRINT("Received a MAD\n");
	status = iba_umadt_release_recvmad(umadtHandle, Mad);
	if (status != FSUCCESS) {
		fprintf(stderr, "%s: ReleaseRecvMad: (status=0x%x): %s\n",
						g_Top_cmdname, status, FSTATUS_MSG(status));
	}
	return status;
}

static void flush_rcv(MADT_HANDLE umadtHandle, int timeout)
{
	FSTATUS status;
	MadtStruct *Mad;
	MadWorkCompletion *Wc;

	if (timeout) {
		status = iba_umadt_wait_any_compl(umadtHandle, RECV_COMPLETION, timeout);
		if (status == FTIMEOUT) {
			return;
		}
	}
	do {
		status = iba_umadt_poll_send_compl(umadtHandle, &Mad, &Wc);
		if (status == FSUCCESS) {
			(void)iba_umadt_release_sendmad(umadtHandle, Mad);
		}
	} while (status == FSUCCESS);
	do {
		// get all the recv's from the Q
		status = iba_umadt_poll_recv_compl(umadtHandle, &Mad, &Wc);
		if (status != FSUCCESS) {
			return;
		}
		DBGPRINT("Received a MAD\n");
		status = iba_umadt_release_recvmad(umadtHandle, Mad);
		if (status != FSUCCESS) {
			fprintf(stderr, "%s: ReleaseRecvMad: (status=0x%x): %s\n",
							g_Top_cmdname, status, FSTATUS_MSG(status));
		}
	} while (1);
}

FSTATUS send_recv_mad(MADT_HANDLE umadtHandle, IB_PATH_RECORD *pathp,
						uint32 qpn, uint32 qkey, MAD* mad,
						int timeout, int retries)
{
	FSTATUS fstatus;
	int attempts =  retries+1;
	MAD req = *mad;

	do {
		fstatus = send_mad(g_umadtHandle, pathp, qpn, qkey, &req);
		if (fstatus == FSUCCESS) {
			do {
				fstatus = recv_mad(g_umadtHandle, (MAD*)mad, timeout);
				if (fstatus != FSUCCESS)
				{
					DBGPRINT("Failed to get MAD response: %s DLID: %u\n", iba_fstatus_msg(fstatus), pathp->DLID);
					break;
				}
				BSWAP_MAD_HEADER((MAD*)mad);
				if (mad->common.TransactionID != (g_transId<<24))
					DBGPRINT("Recv unexpected trans id: expected 0x%"PRIx64" got 0x%"PRIx64"\n", (g_transId<<24), mad->common.TransactionID);
			} while (mad->common.TransactionID != (g_transId<<24));
		}
	} while (FSUCCESS != fstatus && --attempts);
	BSWAP_MAD_HEADER((MAD*)mad);
	return fstatus;
}
#endif

#ifdef PRODUCT_OPENIB_FF

static __inline__ void debugLogSmaRequest(const char* requestName, uint8_t* path, STL_LID dlid, STL_LID slid) {
	int i;

	if(path) {
		DBGPRINT("Sending DR SMA %s to ", requestName);
		if(slid)
			DBGPRINT("slid: 0x%08x, ", slid);

		DBGPRINT("path:");
		for(i = 1; i <= path[0]; i++) {
			DBGPRINT(" %02d", path[i]);
		}
		DBGPRINT("\n");
	} else {
		DBGPRINT("Sending SMA %s to LID 0x%x\n", requestName, dlid);
	}
}

/**
 * Issue a single SMA mad and get the response.
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param method Request method, likely either get or set
 * @param attr Attribute type being issued for this request
 * @param modifier Attribute modifier for the specified attribute
 * @param buffer Pointer to attribute data
 * @param bufferLen The Length of the attribute data in the buffer
 * @return FSTATUS return code
 */
static FSTATUS stl_sma_send_recv_mad(struct omgt_port *port,
									 STL_LID dlid,
									 STL_LID slid,
									 uint8_t* path,
									 uint8_t method, 
									 uint32_t attr, 
									 uint32_t modifier, 
									 uint8_t* buffer,
									 uint32_t bufferLength)
{
	FSTATUS fstatus;
	STL_SMP smp;
	struct omgt_mad_addr addr;
	size_t send_size;
    size_t recv_size;

	memset(&smp, 0, sizeof(smp));
	memset(&addr, 0, sizeof(addr));

	if(dlid && path == NULL) {
		// LID routed only
		smp.common.MgmtClass = MCLASS_SM_LID_ROUTED;
		smp.common.u.NS.Status.AsReg16 = 0;
		addr.lid = dlid;
	} else if (!dlid && !slid && path) {
		// Directed route only
		addr.lid = STL_LID_PERMISSIVE;
		smp.common.MgmtClass = MCLASS_SM_DIRECTED_ROUTE;
		smp.common.u.DR.s.D = 0;
		smp.common.u.DR.s.Status = 0;
		smp.common.u.DR.HopPointer = 0;
		smp.common.u.DR.HopCount = path[0];
		smp.SmpExt.DirectedRoute.DrSLID = STL_LID_PERMISSIVE;
		smp.SmpExt.DirectedRoute.DrDLID = STL_LID_PERMISSIVE;
		memcpy(smp.SmpExt.DirectedRoute.InitPath, path, sizeof(smp.SmpExt.DirectedRoute.InitPath));
	} else if (!dlid && slid && path) {
		// Mixed LR-DR (initial LID route, then DR)
		addr.lid = STL_LID_PERMISSIVE;
		smp.common.MgmtClass = MCLASS_SM_DIRECTED_ROUTE;
		smp.common.u.DR.s.D = 0;
		smp.common.u.DR.s.Status = 0;
		smp.common.u.DR.HopPointer = 0;
		smp.common.u.DR.HopCount = path[0];
		smp.SmpExt.DirectedRoute.DrSLID = slid;
		smp.SmpExt.DirectedRoute.DrDLID = STL_LID_PERMISSIVE;
		memcpy(smp.SmpExt.DirectedRoute.InitPath, path, sizeof(smp.SmpExt.DirectedRoute.InitPath));
	} else {
		DBGPRINT("ERROR: unable to route packet: slid, dlid, or path not properly specified\n");
		return (FINVALID_PARAMETER);
	}

	smp.common.BaseVersion = STL_BASE_VERSION;
	smp.common.ClassVersion = STL_SM_CLASS_VERSION;
	smp.common.mr.AsReg8 = 0;
	smp.common.mr.s.Method = method;
#if defined(IB_STACK_IBACCESS) || defined(CAL_IBACCESS)
	smp.common.TransactionID = (++g_transId)<<24;
#else
	smp.common.TransactionID = (++g_transId) & 0xffffffff;
#endif
	smp.common.AttributeID = attr;
	smp.common.AttributeModifier = modifier;
	smp.M_Key = g_mkey;

	// Copy the attribute information into the SMP
	memcpy(stl_get_smp_data(&smp), buffer, bufferLength);

    // Determine which pkey to use (full or limited)
    // Attempt to use full at all times, otherwise, can
    // use the limited for queries of the local port.
    uint16_t pkey = omgt_get_mgmt_pkey(port, dlid, 0);
    if (pkey==0) {
        DBGPRINT("ERROR: Local port does not have management privileges\n");
        return (FPROTECTION);
    }

	addr.qpn = 0;
	addr.qkey = 0;
	addr.pkey = pkey;

	send_size = bufferLength;
    send_size += stl_get_smp_header_size(&smp);
    send_size = ROUNDUP_TYPE(size_t, send_size, 8);
	STL_BSWAP_SMP_HEADER(&smp);

#ifdef IB_DEBUG
	DBGPRINT("Sending STL MAD:\n");
	DumpMad(&smp);
#endif

    recv_size = sizeof(smp);
	fstatus = omgt_send_recv_mad_no_alloc(port, 
										 (uint8_t *)&smp, send_size, 
										 &addr,
										 (uint8_t *)&smp, &recv_size,
										 RESP_WAIT_TIME, 
										 g_smaRetries-1);
#ifdef IB_DEBUG
	if (fstatus == FSUCCESS) {
		DBGPRINT("Received STL MAD:\n");
		StlDumpMad(&smp);
	}
#endif
	STL_BSWAP_SMP_HEADER(&smp);

	if (smp.common.MgmtClass == MCLASS_SM_DIRECTED_ROUTE && path && memcmp(path, smp.SmpExt.DirectedRoute.InitPath, sizeof(smp.SmpExt.DirectedRoute.InitPath)) != 0) {
		int i;

		DBGPRINT("Response failed directed route validation, received packet with path: ");
		for(i = 1; i < 64; i++) {
			if(smp.SmpExt.DirectedRoute.InitPath[i] != 0) {
				DBGPRINT("%d ", smp.SmpExt.DirectedRoute.InitPath[i]);
			} else {
				break;
			}
		}
		DBGPRINT("\n");

		fstatus = FERROR;
	}

	if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
		DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
		fstatus = FERROR;
	} else {
		memcpy(buffer, stl_get_smp_data(&smp), bufferLength);
	}

	return fstatus;
}

/**
 * Get the NodeDesc from the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param pNodeDesc Pointer to allocated space to store the returned NodeDesc
 * @return FSTATUS return code
 */
FSTATUS SmaGetNodeDesc(struct omgt_port *port, 
					   STL_LID dlid, 
					   STL_LID slid,
					   uint8_t* path,
					   STL_NODE_DESCRIPTION *pNodeDesc)
{
	FSTATUS fstatus;
	uint32_t bufferLength = sizeof(STL_NODE_DESCRIPTION);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	debugLogSmaRequest("Get(NodeDesc)", path, dlid, slid);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_NODE_DESCRIPTION, 0, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_NODE_DESCRIPTION((STL_NODE_DESCRIPTION*)buffer);
		memcpy(pNodeDesc, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Get the NodeInfo from the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param pNodeInfo Pointer to allocated space to store the returned NodeInfo
 * @return FSTATUS return code
 */
FSTATUS SmaGetNodeInfo(struct omgt_port *port, 
					   STL_LID dlid, 
					   STL_LID slid,
					   uint8_t* path,
					   STL_NODE_INFO *pNodeInfo)
{
	FSTATUS fstatus;
	uint32_t bufferLength = sizeof(STL_NODE_INFO);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	debugLogSmaRequest("Get(NodeInfo)", path, dlid, slid);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_NODE_INFO, 0, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_NODE_INFO((STL_NODE_INFO*)buffer);
		memcpy(pNodeInfo, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Get the SwitchInfo from the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param pSwitchInfo Pointer to allocated space to store the returned SwitchInfo
 * @return FSTATUS return code
 */
FSTATUS SmaGetSwitchInfo(struct omgt_port *port, 
						 STL_LID dlid,
						 STL_LID slid,
						 uint8_t* path,
						 STL_SWITCH_INFO *pSwitchInfo)
{
	FSTATUS fstatus;
	uint32_t bufferLength = sizeof(STL_SWITCH_INFO);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	debugLogSmaRequest("Get(SwitchInfo)", path, dlid, slid);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_SWITCH_INFO, 0, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_SWITCH_INFO((STL_SWITCH_INFO*)buffer);
		memcpy(pSwitchInfo, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Get the PortInfo from the requested LID/path and portNum
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param portNum Port number for desired port information
 * @param pPortInfo Pointer to allocated space to store the returned PortInfo
 * @return FSTATUS return code
 */
FSTATUS SmaGetPortInfo(struct omgt_port *port,
					   STL_LID dlid, 
					   STL_LID slid,
					   uint8_t* path,
					   uint8_t portNum, 
					   uint8_t smConfigStarted,
					   STL_PORT_INFO *pPortInfo)
{
	FSTATUS fstatus;
	uint32 amod = 0x01000000 | portNum | (smConfigStarted ? 0x00000200 : 0x0);
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_PORT_INFO);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Get(PortInfo %u)", portNum);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_PORT_INFO, amod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_PORT_INFO((STL_PORT_INFO*)buffer);
		memcpy(pPortInfo, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Get the CableInfo from the requested LID/path starting at address addr with length len
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param portNum Port number for desired port information
 * @param addr Starting address for CableInfo
 * @param len Length of requested CableInfo data
 * @param data Pointer to allocated space to store the returned CableInfo data
 * @return FSTATUS return code
 */
FSTATUS SmaGetCableInfo(struct omgt_port *port,
						STL_LID dlid, 
						STL_LID slid,
						uint8_t* path,
						uint8_t portNum,
						uint16_t addr, 
						uint8_t len,
						uint8_t *data)
{
	FSTATUS fstatus;
	uint32_t amod = (addr & 0x07ff)<<19 | (len & 0x3f)<<13 | (portNum & 0xff);
	STL_CABLE_INFO *pCableInfo;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_CABLE_INFO);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Get(CableInfo %u)", portNum);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_CABLE_INFO, amod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		pCableInfo = (STL_CABLE_INFO*)buffer;
		BSWAP_STL_CABLE_INFO(pCableInfo);
		memcpy(data, pCableInfo->Data, len+1);
	}
	return fstatus;
}

/**
 * Get the Partition Table from the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param portNum Port number for desired port information
 * @param block PKey table block number
 * @param pPartTable Pointer to allocated space to store the returned Partition Table information
 * @return FSTATUS return code
 */
FSTATUS SmaGetPartTable(struct omgt_port *port,
						STL_LID dlid, 
						STL_LID slid,
						uint8_t* path,
						uint8_t portNum, 
						uint16_t block, 
						STL_PARTITION_TABLE *pPartTable)
{
	FSTATUS fstatus;
	uint32 attrmod = (1<<24) | (portNum<<16) | block;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_PARTITION_TABLE);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Get(P_KeyTable %u %u)", portNum, block);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_PART_TABLE, attrmod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_PARTITION_TABLE((STL_PARTITION_TABLE*)buffer);
		memcpy(pPartTable, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Get the VLArb Table from the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param portNum Port number for desired port information
 * @param part Which part of the VLArb table to retrieve
 * @param pVLArbTable Pointer to allocated space to store the returned VLArb Table information
 * @return FSTATUS return code
 */
FSTATUS SmaGetVLArbTable(struct omgt_port *port,
						 STL_LID dlid,
						 STL_LID slid,
						 uint8_t* path, 
						 uint8_t portNum, 
						 uint8_t part, 
						 STL_VLARB_TABLE *pVLArbTable)
{
	FSTATUS fstatus;
	uint32 attrmod = (1<<24) | (part<<16) | portNum;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_VLARB_TABLE);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Get(VLArb %u %u)", part, portNum);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_VL_ARBITRATION, attrmod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_VLARB_TABLE((STL_VLARB_TABLE*)buffer, part);
		memcpy(pVLArbTable, buffer, bufferLength);
	}
	return fstatus;
}


/**
 * Get the SLSC Mapping Table from the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param pSLSCMap Pointer to allocated space to store the returned SLSC Mapping Table
 * @return FSTATUS return code
 */
FSTATUS SmaGetSLSCMappingTable(struct omgt_port *port,
							   STL_LID dlid, 
							   STL_LID slid,
							   uint8_t* path,
							   STL_SLSCMAP *pSLSCMap)
{
	FSTATUS fstatus;
	uint32 attrmod = 0;
	uint32_t bufferLength = sizeof(STL_SLSCMAP);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	debugLogSmaRequest("Get(SLSCMap)", path, dlid, slid);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_SL_SC_MAPPING_TABLE, attrmod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_SLSCMAP((STL_SLSCMAP*)buffer);
		memcpy(pSLSCMap, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Get the SCSL Mapping Table from the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param pSCSLMap Pointer to allocated space to store the returned SCSL Mapping Table
 * @return FSTATUS return code
 */
FSTATUS SmaGetSCSLMappingTable(struct omgt_port *port,
							   STL_LID dlid, 
							   STL_LID slid,
							   uint8_t* path,
							   STL_SCSLMAP *pSCSLMap)
{
	FSTATUS fstatus;
	uint32 attrmod = 0;
	uint32_t bufferLength = sizeof(STL_SCSLMAP);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	debugLogSmaRequest("Get(SCSLMap)", path, dlid, slid);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_SC_SL_MAPPING_TABLE, attrmod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_SCSLMAP((STL_SCSLMAP*)buffer);
		memcpy(pSCSLMap, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Get the SCSC Mapping Table from the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param in_port Ingress port number for SCSC Mapping Table
 * @param out_port Egress port number for SCSC Mapping Table
 * @param pSCSCMap Pointer to allocated space to store the returned SCSC Mapping Table
 * @return FSTATUS return code
 */
FSTATUS SmaGetSCSCMappingTable(struct omgt_port *port,
							   STL_LID dlid, 
							   STL_LID slid,
							   uint8_t* path,
							   uint8_t in_port, 
							   uint8_t out_port, 
							   STL_SCSCMAP *pSCSCMap)
{
	FSTATUS fstatus;
	uint32 attrmod = (1 << 24) | (in_port<<8) | out_port;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_SCSCMAP);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Get(SCSCMap %u %u)", in_port, out_port);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_SC_SC_MAPPING_TABLE, attrmod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_SCSCMAP((STL_SCSCMAP*)buffer);
		memcpy(pSCSCMap, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Get the SCVL Mapping Table from the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param portNum Port number for desired port information
 * @param pSCVLMap Pointer to allocated space to store the returned SCVL Mapping Table
 * @param attr SMP Attribute value - used to select between different SCVL Mapping Tables
 * @return FSTATUS return code
 */
FSTATUS SmaGetSCVLMappingTable(struct omgt_port *port,
							   STL_LID dlid, 
							   STL_LID slid,
							   uint8_t* path,
							   uint8_t port_num,
							   STL_SCVLMAP *pSCVLMap,
							   uint16_t attr)
{
	FSTATUS fstatus;
	uint32 attrmod = (1<<24) | port_num;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_SCVLMAP);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Get(SCVLMap %u)", port_num);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, attr, attrmod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_SCVLMAP((STL_SCVLMAP*)buffer);
		memcpy(pSCVLMap, buffer, bufferLength);
	}
	return fstatus;  
}

/**
 * Get the Buffer Control Table from the requested LID/path
 * Retry as needed if unable to send or don't get a response
 * pBCT must be large enough to hold all BufferControlTables
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param startPort Beginning of port range for returned Buffer Control Tables
 * @param endPort End of port range for returned Buffer Control Tables
 * @param pBCT[] Array to store the returned Buffer Control Table(s)
 * @return FSTATUS return code
 */
FSTATUS SmaGetBufferControlTable(struct omgt_port *port,
							   STL_LID dlid, 
							   STL_LID slid,
							   uint8_t* path,
							   uint8_t startPort,
							   uint8_t endPort,
							   STL_BUFFER_CONTROL_TABLE pBCT[])
{
	FSTATUS fstatus = FERROR;
    uint8_t maxCount = path == NULL ? STL_NUM_BFRCTLTAB_BLOCKS_PER_LID_SMP : STL_NUM_BFRCTLTAB_BLOCKS_PER_DRSMP;
	uint8_t block;
	char attributeName[64];
	const size_t bufferLength = STL_BFRCTRLTAB_PAD_SIZE * maxCount;
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Get(BufferControlTable %u %u)", startPort, endPort);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	for (block = startPort; block <= endPort; block += maxCount) {
		uint8_t numPorts = MIN(maxCount, (endPort - block)+1);
		uint32_t amod = (numPorts << 24) | block;

		fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_BUFFER_CONTROL_TABLE, amod, buffer, bufferLength);

		if (fstatus == FSUCCESS) {
			int i;
			STL_BUFFER_CONTROL_TABLE *table = (STL_BUFFER_CONTROL_TABLE *)buffer;

			for (i = 0; i < numPorts; i++) {
				BSWAP_STL_BUFFER_CONTROL_TABLE(table);
				pBCT[block-startPort+i] = *table;
				// Handle the dissimilar sizes of Buffer Table and 8-byte pad alignment
				table = (STL_BUFFER_CONTROL_TABLE *)((uint8_t *)table + STL_BFRCTRLTAB_PAD_SIZE);
			}
		}
	}

	return fstatus;
}

/**
 * Get the Linear Forwarding Table from the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param block Linear Forwarding Table block number
 * @param pFDB Pointer to allocated space to store the returned Linear Forwarding Table
 * @return FSTATUS return code
 */
FSTATUS SmaGetLinearFDBTable(struct omgt_port *port,
							 STL_LID dlid, 
							 STL_LID slid,
							 uint8_t* path,
							 uint16_t block, 
							 STL_LINEAR_FORWARDING_TABLE *pFDB)
{
	FSTATUS fstatus;
	uint32_t modifier;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_LINEAR_FORWARDING_TABLE);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Get(LFT %u)", block);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	modifier = 0x01000000 + (uint32_t)block;

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_LINEAR_FWD_TABLE, modifier, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_LINEAR_FORWARDING_TABLE((STL_LINEAR_FORWARDING_TABLE*)buffer);
		memcpy(pFDB, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Get the Multicast Forwarding Table from the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param block Multicast Forwarding Table block number
 * @param position Desired port mask for selected block (0-3)
 * @param pFDB Pointer to allocated space to store the returned Multicast Forwarding Table
 * @return FSTATUS return code
 */
FSTATUS SmaGetMulticastFDBTable(struct omgt_port *port,
							    STL_LID dlid, 
							    STL_LID slid,
							    uint8_t* path,
								uint32_t block, 
								uint8_t position, 
								STL_MULTICAST_FORWARDING_TABLE *pFDB)
{
	FSTATUS fstatus;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_MULTICAST_FORWARDING_TABLE);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Get(MFT %u %u)", block, position);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	//@TODO: Enable multi-block requests from just a single block request
	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_MCAST_FWD_TABLE, 
									(0x1<<24) | (0x3 & position)<<22 | (block & 0xfffff), buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_MULTICAST_FORWARDING_TABLE((STL_MULTICAST_FORWARDING_TABLE*)buffer);
		memcpy(pFDB, buffer, bufferLength);
	}
	return fstatus;
}

/* Get PortGroup Table from SMA at lid
 * Retry as needed
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param block Port Group Table block number
 * @param pPGT Pointer to allocated space to store the returned Port Group Table
 * @return FSTATUS return code
 */
FSTATUS SmaGetPortGroupTable(struct omgt_port *port,
							 STL_LID dlid,
							 STL_LID slid,
							 uint8_t* path,
							 uint16 block,
							 STL_PORT_GROUP_TABLE *pPGT)
{
	FSTATUS fstatus;
	uint32_t modifier;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_PORT_GROUP_TABLE);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Get(PGT %u)", block);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	modifier = 0x01000000 + (uint32_t)block;

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_PORT_GROUP_TABLE,
									modifier, buffer, bufferLength);
	if (FSUCCESS == fstatus) {
		BSWAP_STL_PORT_GROUP_TABLE((STL_PORT_GROUP_TABLE*)buffer);
		memcpy(pPGT, buffer, bufferLength);
	}
	return fstatus;
}

/* Get PortGroup FDB Table from SMA at lid
 * Retry as needed
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param block Port Group FDB Table block number
 * @param pFDB Pointer to allocated space to store the returned Port Group FDB Table
 * @return FSTATUS return code
 */
FSTATUS SmaGetPortGroupFDBTable(struct omgt_port *port,
							 STL_LID dlid,
							 STL_LID slid,
							 uint8_t* path,
							 uint16 block,
							 STL_PORT_GROUP_FORWARDING_TABLE *pFDB)
{
	FSTATUS fstatus;
	uint32_t modifier;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_PORT_GROUP_TABLE);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Get(PGFDB %u)", block);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	modifier = 0x01000000 + (uint32_t)block;
	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_PORT_GROUP_FWD_TABLE,
									modifier, buffer, bufferLength);
	if (FSUCCESS == fstatus) {
		BSWAP_STL_PORT_GROUP_FORWARDING_TABLE((STL_PORT_GROUP_FORWARDING_TABLE*)buffer);
		memcpy(pFDB, buffer, bufferLength);
	}
	return fstatus;
}



/**
 * Set the SwitchInfo of the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param pSwitchInfo Pointer to SwitchInfo to set. Will be overwritten with response
 * @return FSTATUS return code
 */
FSTATUS SmaSetSwitchInfo(struct omgt_port *port, 
						 STL_LID dlid,
						 STL_LID slid,
						 uint8_t* path,
						 STL_SWITCH_INFO *pSwitchInfo)
{
	FSTATUS fstatus;
	uint32_t bufferLength = sizeof(STL_SWITCH_INFO);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	debugLogSmaRequest("Set(SwitchInfo)", path, dlid, slid);
	
	memcpy(buffer, pSwitchInfo, bufferLength);
	BSWAP_STL_SWITCH_INFO((STL_SWITCH_INFO*)buffer);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_SET, STL_MCLASS_ATTRIB_ID_SWITCH_INFO, 0, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_SWITCH_INFO((STL_SWITCH_INFO*)buffer);
		memcpy(pSwitchInfo, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Set the PortInfo of the requested LID/path and portNum
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param portNum Port number for desired port information
 * @param pPortInfo Pointer to PortInfo to set. Will be overwritten with response
 * @return FSTATUS return code
 */
FSTATUS SmaSetPortInfo(struct omgt_port *port,
					   STL_LID dlid, 
					   STL_LID slid,
					   uint8_t* path,
					   uint8_t portNum, 
					   STL_PORT_INFO *pPortInfo)
{
	FSTATUS fstatus;
	uint32 amod = 0x01000000 | portNum;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_PORT_INFO);
	uint8_t buffer[bufferLength];
	
	snprintf(attributeName, sizeof(attributeName), "Set(PortInfo %u)", portNum);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	memcpy(buffer, pPortInfo, bufferLength);
	BSWAP_STL_PORT_INFO((STL_PORT_INFO*)buffer);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_SET, STL_MCLASS_ATTRIB_ID_PORT_INFO, amod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_PORT_INFO((STL_PORT_INFO*)buffer);
		memcpy(pPortInfo, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Set the Partition Table of the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param portNum Port number for desired port information
 * @param block PKey table block number
 * @param pPartTable Pointer to Partition table to set. Will be overwritten with response
 * @return FSTATUS return code
 */
FSTATUS SmaSetPartTable(struct omgt_port *port,
						STL_LID dlid, 
						STL_LID slid,
						uint8_t* path,
						uint8_t portNum, 
						uint16_t block, 
						STL_PARTITION_TABLE *pPartTable)
{
	FSTATUS fstatus;
	uint32 attrmod = (1<<24) | (portNum<<16) | block;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_PARTITION_TABLE);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Set(P_KeyTable %u %u)", portNum, block);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	memcpy(buffer, pPartTable, bufferLength);
	BSWAP_STL_PARTITION_TABLE((STL_PARTITION_TABLE*)buffer);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_SET, STL_MCLASS_ATTRIB_ID_PART_TABLE, attrmod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_PARTITION_TABLE((STL_PARTITION_TABLE*)buffer);
		memcpy(pPartTable, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Set the VLArb Table of the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param portNum Port number for desired port information
 * @param part Which part of the VLArb table to write to
 * @param pVLArbTable Pointer to VLArb table to set. Will be overwritten with response
 * @return FSTATUS return code
 */
FSTATUS SmaSetVLArbTable(struct omgt_port *port,
						 STL_LID dlid,
						 STL_LID slid,
						 uint8_t* path, 
						 uint8_t portNum, 
						 uint8_t part, 
						 STL_VLARB_TABLE *pVLArbTable)
{
	FSTATUS fstatus;
	uint32 attrmod = (1<<24) | (part<<16) | portNum;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_VLARB_TABLE);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Set(VLArb %u %u)", part, portNum);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	memcpy(buffer, pVLArbTable, bufferLength);
	BSWAP_STL_VLARB_TABLE((STL_VLARB_TABLE*)buffer, part);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_SET, STL_MCLASS_ATTRIB_ID_VL_ARBITRATION, attrmod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_VLARB_TABLE((STL_VLARB_TABLE*)buffer, part);
		memcpy(pVLArbTable, buffer, bufferLength);
	}
	return fstatus;
}


/**
 * Set the SLSC Mapping Table of the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param pSLSCMap Pointer to SLSC Mapping Table to set. Will be overwritten with response
 * @return FSTATUS return code
 */
FSTATUS SmaSetSLSCMappingTable(struct omgt_port *port,
							   STL_LID dlid, 
							   STL_LID slid,
							   uint8_t* path,
							   STL_SLSCMAP *pSLSCMap)
{
	FSTATUS fstatus;
	uint32 attrmod = 0;
	uint32_t bufferLength = sizeof(STL_SLSCMAP);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	debugLogSmaRequest("Set(SLSCMap)", path, dlid, slid);

	memcpy(buffer, pSLSCMap, bufferLength);
	BSWAP_STL_SLSCMAP((STL_SLSCMAP*)buffer);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_SET, STL_MCLASS_ATTRIB_ID_SL_SC_MAPPING_TABLE, attrmod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_SLSCMAP((STL_SLSCMAP*)buffer);
		memcpy(pSLSCMap, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Set the SCSL Mapping Table of the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param pSCSLMap Pointer to SCSL Mapping Table to set. Will be overwritten with response
 * @return FSTATUS return code
 */
FSTATUS SmaSetSCSLMappingTable(struct omgt_port *port,
							   STL_LID dlid, 
							   STL_LID slid,
							   uint8_t* path,
							   STL_SCSLMAP *pSCSLMap)
{
	FSTATUS fstatus;
	uint32 attrmod = 0;
	uint32_t bufferLength = sizeof(STL_SCSLMAP);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	debugLogSmaRequest("Set(SCSLMap)", path, dlid, slid);

	memcpy(buffer, pSCSLMap, bufferLength);
	BSWAP_STL_SCSLMAP((STL_SCSLMAP*)buffer);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_SET, STL_MCLASS_ATTRIB_ID_SC_SL_MAPPING_TABLE, attrmod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_SCSLMAP((STL_SCSLMAP*)buffer);
		memcpy(pSCSLMap, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Set the SCSC Mapping Table of the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param in_port Ingress port number for SCSC Mapping Table
 * @param out_port Egress port number for SCSC Mapping Table
 * @param pSCSCMap Pointer to SCSC Mapping Table to set. Will be overwritten with response
 * @return FSTATUS return code
 */
FSTATUS SmaSetSCSCMappingTable(struct omgt_port *port,
							   STL_LID dlid, 
							   STL_LID slid,
							   uint8_t* path,
							   uint8_t in_port, 
							   uint8_t out_port, 
							   STL_SCSCMAP *pSCSCMap)
{
	FSTATUS fstatus;
	uint32 attrmod = (1 << 24) | (in_port<<8) | out_port;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_SCSCMAP);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Set(SCSCMap %u %u)", in_port, out_port);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	memcpy(buffer, pSCSCMap, bufferLength);
	BSWAP_STL_SCSCMAP((STL_SCSCMAP*)buffer);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_SET, STL_MCLASS_ATTRIB_ID_SC_SC_MAPPING_TABLE, attrmod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_SCSCMAP((STL_SCSCMAP*)buffer);
		memcpy(pSCSCMap, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Set the SCVL Mapping Table of the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param asyncUpdate Asynchronous update
 * @param allPorts All ports starting at portNum
 * @param portNum Port number for desired port information
 * @param pSCVLMap Pointer to SCVL Mapping Table to set. Will be overwritten with response
 * @param attr SMP Attribute value - used to select between different SCVL Mapping Tables
 * @return FSTATUS return code
 */
FSTATUS SmaSetSCVLMappingTable(struct omgt_port *port,
							   STL_LID dlid, 
							   STL_LID slid,
							   uint8_t* path,
							   boolean asyncUpdate,
							   boolean allPorts,
							   uint8_t portNum,
							   STL_SCVLMAP *pSCVLMap,
							   uint16_t attr)
{
	FSTATUS fstatus;
	uint32 attrmod = (1<<24) | portNum;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_SCVLMAP);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	if (allPorts)
		attrmod |= 1 << 8;

	if (asyncUpdate)
		attrmod |= 1 << 12;

	snprintf(attributeName, sizeof(attributeName), "Set(SCVLMap %u)", portNum);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	memcpy(buffer, pSCVLMap, bufferLength);
	BSWAP_STL_SCVLMAP((STL_SCVLMAP*)buffer);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_SET, attr, attrmod, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_SCVLMAP((STL_SCVLMAP*)buffer);
		memcpy(pSCVLMap, buffer, bufferLength);
	}
	return fstatus;  
}

/**
 * Set the Buffer Control Table from the requested LID/path
 * Retry as needed if unable to send or don't get a response
 * pBCT must be large enough to hold all BufferControlTables
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param startPort Beginning of port range for returned Buffer Control Tables
 * @param endPort End of port range for returned Buffer Control Tables
 * @param pBCT[] Array of Buffer Control Table(s) to set. Will be overwritten with response
 * @return FSTATUS return code
 */
FSTATUS SmaSetBufferControlTable(struct omgt_port *port,
							   STL_LID dlid, 
							   STL_LID slid,
							   uint8_t* path,
							   uint8_t startPort,
							   uint8_t endPort,
							   STL_BUFFER_CONTROL_TABLE pBCT[])
{
	FSTATUS fstatus = FERROR;
    uint8_t maxCount = (path == NULL ? STL_NUM_BFRCTLTAB_BLOCKS_PER_LID_SMP : STL_NUM_BFRCTLTAB_BLOCKS_PER_DRSMP);
	uint8_t block;
	int i;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_BUFFER_CONTROL_TABLE) * maxCount;
	uint8_t buffer[bufferLength];
	uint8_t* data = buffer;
	STL_BUFFER_CONTROL_TABLE *table = (STL_BUFFER_CONTROL_TABLE *)data;
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Set(BufferControlTable %u %u)", startPort, endPort);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	for (block = startPort; block <= endPort; block += maxCount) {
		uint8_t numPorts = MIN(maxCount, (endPort - block)+1);
		uint32_t amod = (numPorts << 24) | block;

		for (i = 0; i < numPorts; i++) {
			memcpy(table, &pBCT[block-startPort+i], sizeof(STL_BUFFER_CONTROL_TABLE));
			BSWAP_STL_BUFFER_CONTROL_TABLE(table);
			// Handle the dissimilar sizes of Buffer Table and 8-byte pad alignment
			data += STL_BFRCTRLTAB_PAD_SIZE;
			table = (STL_BUFFER_CONTROL_TABLE *)(data);
		}

		fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_SET, STL_MCLASS_ATTRIB_ID_BUFFER_CONTROL_TABLE, amod, buffer, sizeof(STL_BUFFER_CONTROL_TABLE));

		if (fstatus == FSUCCESS) {
			data = buffer;
			table = (STL_BUFFER_CONTROL_TABLE*)data;

			for (i = 0; i < numPorts; i++) {
				BSWAP_STL_BUFFER_CONTROL_TABLE(table);
				memcpy(&pBCT[block-startPort+i], table, sizeof(STL_BUFFER_CONTROL_TABLE));
				// Handle the dissimilar sizes of Buffer Table and 8-byte pad alignment
				data += STL_BFRCTRLTAB_PAD_SIZE;
				table = (STL_BUFFER_CONTROL_TABLE *)(data);
			}
		}
	}

	return fstatus;
}

/**
 * Set the Linear Forwarding Table of the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param block Linear Forwarding Table block number
 * @param pFDB Pointer to Linear Forwarding Table to set. Will be overwritten with response
 * @return FSTATUS return code
 */
FSTATUS SmaSetLinearFDBTable(struct omgt_port *port,
							 STL_LID dlid, 
							 STL_LID slid,
							 uint8_t* path,
							 uint16_t block, 
							 STL_LINEAR_FORWARDING_TABLE *pFDB)
{
	FSTATUS fstatus;
	uint32_t modifier;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_LINEAR_FORWARDING_TABLE);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Set(LFT %u)", block);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	modifier = 0x01000000 + (uint32_t)block;

	memcpy(buffer, pFDB, bufferLength);
	BSWAP_STL_LINEAR_FORWARDING_TABLE((STL_LINEAR_FORWARDING_TABLE*)buffer);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_SET, STL_MCLASS_ATTRIB_ID_LINEAR_FWD_TABLE, modifier, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_LINEAR_FORWARDING_TABLE((STL_LINEAR_FORWARDING_TABLE*)buffer);
		memcpy(pFDB, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Set the Multicast Forwarding Table of the requested LID/path
 * Retry as needed if unable to send or don't get a response
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param block Multicast Forwarding Table block number
 * @param position Desired port mask for selected block (0-3)
 * @param pFDB Pointer to Multicast Forwarding Table to set. Will be overwritten with response
 * @return FSTATUS return code
 */
FSTATUS SmaSetMulticastFDBTable(struct omgt_port *port,
							    STL_LID dlid, 
							    STL_LID slid,
							    uint8_t* path,
								uint32_t block, 
								uint8_t position, 
								STL_MULTICAST_FORWARDING_TABLE *pFDB)
{
	FSTATUS fstatus;
	char attributeName[64];
	uint32_t bufferLength = sizeof(STL_MULTICAST_FORWARDING_TABLE);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	snprintf(attributeName, sizeof(attributeName), "Set(MFT %u %u)", block, position);
	debugLogSmaRequest(attributeName, path, dlid, slid);

	memcpy(buffer, pFDB, bufferLength);
	BSWAP_STL_MULTICAST_FORWARDING_TABLE((STL_MULTICAST_FORWARDING_TABLE*)buffer);

	//@TODO: Enable multi-block requests from just a single block request
	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_SET, STL_MCLASS_ATTRIB_ID_MCAST_FWD_TABLE, 
									(0x1<<24) | (0x3 & position)<<22 | (block & 0xfffff), buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_MULTICAST_FORWARDING_TABLE((STL_MULTICAST_FORWARDING_TABLE*)buffer);
		memcpy(pFDB, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Get the SmaGetCongestionInfo from the requested LID/path
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param pCongestionInfo Pointer to allocated space to store the returned CongestionInfo
 * @return FSTATUS return code
 */
FSTATUS SmaGetCongestionInfo(struct omgt_port *port,
					   STL_LID dlid,
					   STL_LID slid,
					   uint8_t* path,
					   STL_CONGESTION_INFO *pCongestionInfo)
{
	FSTATUS fstatus;
	uint32_t bufferLength = sizeof(STL_CONGESTION_INFO);
	uint8_t buffer[bufferLength];
	memset(buffer, 0, bufferLength);

	debugLogSmaRequest("Get(CongestionInfo)", path, dlid, slid);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_CONGESTION_INFO, 0, buffer, bufferLength);

	if (fstatus == FSUCCESS) {
		BSWAP_STL_CONGESTION_INFO((STL_CONGESTION_INFO*)buffer);
		memcpy(pCongestionInfo, buffer, bufferLength);
	}
	return fstatus;
}

/**
 * Get the SmaGetHFICongestionControlTable from the requested LID/path
 *
 * @param port The omgt_port to communicate with the fabric
 * @param dlid Destination LID to send packet to
 * @param slid Source LID of mixed LRDR packet. Path describes hops after reaching this LID
 * @param path Directed route path to destination
 * @param block Starting block value
 * @param numBlocks Number of blocks in table
 * @return FSTATUS return code
 */

/* Maximum HFICCT size that can fit in a MAD packet */
#define HFICCTI_MAX_BLOCK 14
FSTATUS SmaGetHFICongestionControlTable(struct omgt_port *port,
					   STL_LID dlid,
					   STL_LID slid,
					   uint8_t* path,
					   uint16_t block,
					   uint16_t numBlocks,
					   STL_HFI_CONGESTION_CONTROL_TABLE *pHfiCongestionControl)
{
	FSTATUS fstatus;

	if (numBlocks < 1 || numBlocks > HFICCTI_MAX_BLOCK)
		return FERROR;

	uint32_t bufferLength = sizeof(STL_HFI_CONGESTION_CONTROL_TABLE) +
				sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK) * (numBlocks - 1);

	uint8_t buffer[bufferLength];
	uint32_t amod = (numBlocks<<24) | (block & 0xff);

	memset(buffer, 0, bufferLength);

	debugLogSmaRequest("Get(HFICongestionControlTable)", path, dlid, slid);

	fstatus = stl_sma_send_recv_mad(port, dlid, slid, path, MMTHD_GET, STL_MCLASS_ATTRIB_ID_HFI_CONGESTION_CONTROL_TABLE, amod, buffer, bufferLength);

	if (fstatus != FSUCCESS)
		return fstatus;

	BSWAP_STL_HFI_CONGESTION_CONTROL_TABLE((STL_HFI_CONGESTION_CONTROL_TABLE*)buffer, numBlocks);
	memcpy(pHfiCongestionControl, buffer, bufferLength);

	return FSUCCESS;
}

#endif	// PRODUCT_OPENIB_FF

/* process a PMA class port info or redirection required response
 * save capability mask and update redirectp as needed
 */
static FSTATUS ProcessPmaClassPortInfo(PortData* portp, STL_CLASS_PORT_INFO *classp,
						IB_PATH_RECORD *orig_pathp)
{
	portp->PmaGotClassPortInfo = 1;
	BSWAP_STL_CLASS_PORT_INFO(classp);

	DBGPRINT("PMA ClassPortInfo.CapMask = 0x%x\n", classp->CapMask);
	return FSUCCESS;
}

/* issue a single PMA mad and get the response.
 * Retry as needed if unable to send or don't get a response
 */
static FSTATUS stl_pm_send_recv_mad(struct omgt_port *port, IB_PATH_RECORD *pathp,
	uint32 qpn, uint32 qkey, uint8 method, uint32 attr, uint32 modifier, STL_PERF_MAD *mad)
{
	FSTATUS fstatus;
	struct omgt_mad_addr addr;
    size_t recv_size;

	memset(&addr, 0, sizeof(addr));
	addr.lid = pathp->DLID;
	addr.qpn = qpn;
	addr.qkey = qkey;
	addr.pkey = pathp->P_Key;
	addr.sl = pathp->u2.s.SL;

	mad->common.BaseVersion = STL_BASE_VERSION;
	mad->common.MgmtClass = MCLASS_PERF;
	mad->common.ClassVersion = STL_PM_CLASS_VERSION;
	mad->common.mr.AsReg8 = 0;
	mad->common.mr.s.Method = method;
	mad->common.u.NS.Status.AsReg16 = 0;
#if defined(IB_STACK_IBACCESS) || defined(CAL_IBACCESS)
	mad->common.TransactionID = (++g_transId)<<24;
#else
	mad->common.TransactionID = (++g_transId) & 0xffffffff;
#endif
	mad->common.AttributeID = attr;
	mad->common.AttributeModifier = modifier;
	// rest of fields should be ignored for a Get, zero'ed above
	BSWAP_MAD_HEADER((MAD*)mad);
#ifdef IB_DEBUG
	DBGPRINT("Sending MAD:\n");
	DumpMad(mad);
#endif

	ASSERT(pathp->DLID);

    recv_size = sizeof(*mad);
	fstatus = omgt_send_recv_mad_no_alloc(port, 
										 (uint8_t *)mad, sizeof(*mad),
										 &addr,
										 (uint8_t *)mad, &recv_size,
										 RESP_WAIT_TIME, 
										 g_smaRetries-1);
	
#ifdef IB_DEBUG
	if (fstatus == FSUCCESS) {
		DBGPRINT("Received MAD:\n");
		DumpMad(mad);
	}
#endif
	BSWAP_MAD_HEADER((MAD*)mad);
	//DBGPRINT("send_recv_mad fstatus=%s\n", iba_fstatus_msg(fstatus));
	return fstatus;
}

/* issue a single PMA request and get the response.
 * Retry as needed if unable to send or don't get a response
 * portp is the port to issue PMA request to (can be port 0 of switch), it need
 * not match portNum.
 * Handle PMA redirection (currently not supported) here.
 */
static FSTATUS stl_pm_send_recv(struct omgt_port *port, PortData *portp, uint8 method,
				uint32 attr, uint32 modifier, STL_PERF_MAD *req, STL_PERF_MAD *resp)
{
	FSTATUS fstatus;
	int attempts =  2;	// only a single 2nd attempt with redirection

	do {
		IB_PATH_RECORD *pathp;
		uint32 qpn;
		uint32 qkey;

		pathp = portp->pathp;
		qpn = 1;
		qkey = QP1_WELL_KNOWN_Q_KEY;

		*resp = *req;
		//DBGPRINT("LID=0x%x SL=%d PKey=0x%x\n", pathp->DLID, pathp->u2.s.SL, pathp->P_Key);
		fstatus = stl_pm_send_recv_mad(port, pathp, qpn, qkey, method, attr, modifier, resp);
		if (FSUCCESS != fstatus) {
			//DBGPRINT("pm_send_recv_mad fstatus=%s\n", iba_fstatus_msg(fstatus));
			goto fail;
		}
		if (resp->common.u.NS.Status.AsReg16 != MAD_STATUS_SUCCESS) {
			DBGPRINT("PMA response with bad status: 0x%x\n", resp->common.u.NS.Status.AsReg16);
			fstatus = FERROR;
			goto fail;
		}
	} while (FSUCCESS != fstatus && --attempts);
fail:
	return fstatus;
}

/* Get STL Class Port Info from PMA at portp
 * Retry and handle redirection as needed
 * portp is the port to issue PMA request to (can be port 0 of switch)
 * The portp is updated with the CLASS_PORT_INFO redirect and PMA CapMask
 */
FSTATUS STLPmGetClassPortInfo(struct omgt_port *port, PortData *portp)
{
	STL_PERF_MAD req;
	STL_PERF_MAD resp;
	FSTATUS fstatus;

	if (portp->PmaGotClassPortInfo)
		return FSUCCESS;	// if we already have, no use asking again
	MemoryClear(&req, sizeof(req));

	DBGPRINT("Sending PM Get(ClassPortInfo) to LID 0x%08x Node 0x%016"PRIx64"\n",
				portp->pathp->DLID,
				portp->nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)portp->nodep->NodeDesc.NodeString);
	fstatus = stl_pm_send_recv(port, portp, MMTHD_GET, STL_PM_ATTRIB_ID_CLASS_PORTINFO,
							   0, &req, &resp);
	if (FSUCCESS != fstatus)
		goto fail;
	fstatus = ProcessPmaClassPortInfo(portp, (STL_CLASS_PORT_INFO*)&(resp.PerfData),
				portp->pathp);
fail:
	return fstatus;
}

/* Get Port counters from PMA at portp for given PortNum
 * Retry and handle redirection as needed
 * portp is the port to issue PMA request to (can be port 0 of switch), it need
 */
FSTATUS STLPmGetPortStatus(struct omgt_port *port, PortData *portp, uint8 portNum,
							STL_PORT_STATUS_RSP *pPortStatus)
{
	STL_PERF_MAD req;
	STL_PERF_MAD resp;
	STL_PORT_STATUS_REQ* p = (STL_PORT_STATUS_REQ *)&(req.PerfData);
	FSTATUS fstatus;

	MemoryClear(&req, sizeof(req));
	// rest of fields should be ignored for a Get, zero'ed above
	p->PortNumber = portNum;
	p->VLSelectMask = 0x8001; // only do VLs 15 and 0 for now, we will ignore VL counters for now
	BSWAP_STL_PORT_STATUS_REQ(p);

	DBGPRINT("Sending STL PM Get(PortStatus %d) to LID 0x%04x Node 0x%016"PRIx64"\n",
				portNum, portp->pathp->DLID,
				portp->nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)portp->nodep->NodeDesc.NodeString);
	fstatus = stl_pm_send_recv(port, portp, MMTHD_GET, STL_PM_ATTRIB_ID_PORT_STATUS, 
							   0x01000000, &req, &resp);
	if (FSUCCESS != fstatus)
		goto fail;
	BSWAP_STL_PORT_STATUS_RSP((STL_PORT_STATUS_RSP *)resp.PerfData);
	*pPortStatus = *(STL_PORT_STATUS_RSP *)resp.PerfData;
	DBGPRINT("SendPkts=0x%16"PRIx64"\n", pPortStatus->PortXmitPkts);
fail:
	return fstatus;
}

/* Clear port counters issued to PMA at portp.
 * portp is the port to issue the PMA request to (can be port 0, 
 * thought lastPortIndex MUST be 0 in this case - see below).
 *
 * If lastPortIndex > 0, then it's assumed portp is
 * switch port 0, and send a ClearPortCounters for all ports up to lastPortIndex. 
 * Otherwise assume this is an HFI, and only clear the port at portp.
 * Retry and handle redirection as needed
 * If thresholds is non-NULL, only counters with non-zero threshold are cleared
 * otherwise all counters are cleared
 */
FSTATUS STLPmClearPortCounters(struct omgt_port *port, PortData *portp, uint8 lastPortIndex, 
							   uint32 counterselect)
{
	STL_PERF_MAD req;
	STL_PERF_MAD resp;
	STL_CLEAR_PORT_STATUS* p = (STL_CLEAR_PORT_STATUS *)&(req.PerfData);
	FSTATUS fstatus;
	uint8_t i;
	char debugStr[64] = {'\0'};

	MemoryClear(&req, sizeof(req));

	for (i = 0; i < (lastPortIndex / 64); ++i)
		p->PortSelectMask[3 - i] = ~(0ULL);

	if (lastPortIndex)
		p->PortSelectMask[3 - i] = (1ULL<<(lastPortIndex + 1)) - 1; // All ports up to lastPortIndex masked. 
	else
		p->PortSelectMask[3] = 1<<portp->PortNum; // HFI Case.

	p->CounterSelectMask.AsReg32 = counterselect;

	if (g_verbose_file) {
		if (lastPortIndex)
			sprintf(debugStr, "port range [%d-%d]", portp->PortNum, lastPortIndex); 
		else
			sprintf(debugStr, "port %d", portp->PortNum);
	}
	DBGPRINT("Sending PM Set(PortStatus %s, Sel=0x%04x) to LID 0x%04x Node 0x%016"PRIx64"\n",
				debugStr, p->CounterSelectMask.AsReg32, portp->pathp->DLID,
				portp->nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)portp->nodep->NodeDesc.NodeString);
	BSWAP_STL_CLEAR_PORT_STATUS_REQ(p);
	fstatus = stl_pm_send_recv(port, portp, MMTHD_SET, STL_PM_ATTRIB_ID_CLEAR_PORT_STATUS, 
							   1<<24, &req, &resp);
	if (FSUCCESS != fstatus) {
		//DBGPRINT("stl_pm_send_recv fstatus=%s\n", iba_fstatus_msg(fstatus));
		goto fail;
	}
fail:
	return fstatus;
}


#if !defined(VXWORKS) || defined(BUILD_DMC)
static FSTATUS dm_send_recv(struct omgt_port *port,
							IB_PATH_RECORD *pathp, 
							uint32 attr, 
							uint32 modifier,
							DM_MAD *mad)
{
	FSTATUS fstatus;
	struct omgt_mad_addr addr;
    size_t recv_size;

	memset(&addr, 0, sizeof(addr));
	addr.lid = pathp->DLID;
	if (pathp->u1.s.HopLimit == 1) {
		if ((pathp->DGID.Type.Global.InterfaceID >> 40) == OUI_TRUESCALE)
			addr.lid = pathp->DGID.Type.Global.InterfaceID & 0xFFFFFFFF;
	}
	addr.qpn = 1;
	addr.qkey = QP1_WELL_KNOWN_Q_KEY;
	addr.pkey = pathp->P_Key;

	mad->common.BaseVersion = IB_BASE_VERSION;
	mad->common.MgmtClass = MCLASS_DEV_MGT;
	mad->common.ClassVersion = IB_DEV_MGT_CLASS_VERSION;
	mad->common.mr.AsReg8 = 0;
	mad->common.mr.s.Method = MMTHD_GET;
#if defined(IB_STACK_IBACCESS) || defined(CAL_IBACCESS)
	mad->common.TransactionID = (++g_transId) << 24;
#else
	mad->common.TransactionID = (++g_transId) & 0xffffffff;
#endif
	mad->common.AttributeID = attr;
	mad->common.AttributeModifier = modifier;
	// rest of fields should be ignored for a Get, zero'ed above
	BSWAP_MAD_HEADER((MAD*)mad);

	ASSERT(addr.lid);
    recv_size = sizeof(*mad);
	fstatus = omgt_send_recv_mad_no_alloc(port, 
										 (uint8_t *)mad, sizeof(*mad),
										 &addr,
										 (uint8_t *)mad, &recv_size,
										 RESP_WAIT_TIME, 
										 g_smaRetries-1);

	BSWAP_MAD_HEADER((MAD*)mad);
	if (FSUCCESS == fstatus && mad->common.u.NS.Status.AsReg16 != MAD_STATUS_SUCCESS) {
		DBGPRINT("DMA response with bad status: 0x%x\n", mad->common.u.NS.Status.AsReg16);
		fstatus = FERROR;
	}
	return fstatus;
}

FSTATUS DmGetIouInfo(struct omgt_port *port, IB_PATH_RECORD *pathp, IOUnitInfo *pIouInfo)
{
	DM_MAD mad;
	FSTATUS fstatus;

	MemoryClear(&mad, sizeof(mad));
	DBGPRINT("Sending DM Get(IouInfo) to LID 0x%04x\n", pathp->DLID);
	fstatus = dm_send_recv(port, pathp, DM_ATTRIB_ID_IOUNIT_INFO, 0, &mad);
	if (FSUCCESS != fstatus)
		goto fail;
	*pIouInfo = *(IOUnitInfo*)mad.DMData;
	BSWAP_DM_IOUNIT_INFO(pIouInfo);
fail:
	return fstatus;
}

FSTATUS DmGetIocProfile(struct omgt_port *port, 
						IB_PATH_RECORD *pathp, 
						uint8 slot,
						IOC_PROFILE *pIocProfile)
{
	DM_MAD mad;
	FSTATUS fstatus;

	MemoryClear(&mad, sizeof(mad));
	DBGPRINT("Sending DM Get(IocProfile, %u) to LID 0x%04x\n", slot, pathp->DLID);
	fstatus = dm_send_recv(port, pathp, DM_ATTRIB_ID_IOCONTROLLER_PROFILE, slot, &mad);
	if (FSUCCESS != fstatus)
		goto fail;
	*pIocProfile = *(IOC_PROFILE*)mad.DMData;
	BSWAP_DM_IOC_PROFILE(pIocProfile);
fail:
	return fstatus;
}

/* last - first must be <= 3 */
FSTATUS DmGetServiceEntries(struct omgt_port *port, 
							IB_PATH_RECORD *pathp, 
							uint8 slot,
							uint8 first, 
							uint8 last, 
							IOC_SERVICE *pIocServices)
{
	DM_MAD mad;
	FSTATUS fstatus;
	IOC_SERVICE *p;

	MemoryClear(&mad, sizeof(mad));
	DBGPRINT("Sending DM Get(ServiceEntries, %u, %u-%u) to LID 0x%04x\n",
					slot, first, last, pathp->DLID);
	fstatus = dm_send_recv(port, pathp, DM_ATTRIB_ID_SERVICE_ENTRIES,
					DM_ATTRIB_MODIFIER_SERVICE_ENTRIES(slot, first, last), &mad);
	if (FSUCCESS != fstatus)
		goto fail;
	p = (IOC_SERVICE*)mad.DMData;
	for (; first <= last; first++, pIocServices++,p++) {
		*pIocServices = *p;
		BSWAP_DM_IOC_SERVICE(pIocServices);
	}
fail:
	return fstatus;
}
#endif

FSTATUS InitSmaMkey(uint64 mkey)
{
	g_mkey = mkey;
	return FSUCCESS;
}

FSTATUS InitMad(EUI64 portguid, FILE *verbose_file)
{
		g_verbose_file = verbose_file;
#if defined(IB_STACK_IBACCESS) || defined(CAL_IBACCESS)
		return registerUmadt(portguid, &g_umadtHandle);
#else
		return FSUCCESS;
#endif
}

void DestroyMad(void)
{
#if defined(IB_STACK_IBACCESS) || defined(CAL_IBACCESS)
	if (g_umadtHandle) {
		flush_rcv(g_umadtHandle, FLUSH_WAIT_TIME);
		(void)iba_umadt_deregister(g_umadtHandle);
	}
#endif
}
