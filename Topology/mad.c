/* BEGIN_ICS_COPYRIGHT7 ****************************************

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
#define MAD_ATTEMPTS (8)		// number of request/response attempts

#if defined(IB_STACK_IBACCESS) || defined(CAL_IBACCESS)
static IB_HANDLE g_umadtHandle = NULL;	/* umadt handle for DM interface queries */
#endif

static uint64			g_transId		= 1234;	// transaction id
static FILE 			*g_verbose_file = NULL;	// file for verbose output
static uint64 			g_mkey = 0;				// m_Key for SMA operations

FSTATUS send_recv_mad(MADT_HANDLE umadtHandle, IB_PATH_RECORD *pathp,
                             uint32 qpn, uint32 qkey, MAD* mad,
                             int timeout, int retries);

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
/* issue a single SMA mad and get the response.
 * Retry as needed if unable to send or don't get a response
 */

static FSTATUS stl_sma_send_recv_mad(struct omgt_port *port,
									 uint16_t lid,
									 uint8 method, 
									 uint32 attr, 
									 uint32 modifier, 
									 STL_SMP *smp)
{
	FSTATUS fstatus;
	struct omgt_mad_addr addr;
    size_t recv_size;

    // Determine which pkey to use (full or limited)
    // Attempt to use full at all times, otherwise, can
    // use the limited for queries of the local port.
    uint16_t pkey = omgt_get_mgmt_pkey(port, lid, 0);
    if (pkey==0) {
        DBGPRINT("ERROR: Local port does not have management privileges\n");
        return (FPROTECTION);
    }

	memset(&addr, 0, sizeof(addr));
	addr.lid = lid;
	addr.qpn = 0;
	addr.qkey = 0;
	addr.pkey = pkey;

	smp->common.BaseVersion = STL_BASE_VERSION;
	smp->common.MgmtClass = MCLASS_SM_LID_ROUTED;
	smp->common.ClassVersion = STL_SM_CLASS_VERSION;
	smp->common.mr.AsReg8 = 0;
	smp->common.mr.s.Method = method;
	smp->common.u.NS.Status.AsReg16 = 0;
#if defined(IB_STACK_IBACCESS) || defined(CAL_IBACCESS)
	smp->common.TransactionID = (++g_transId)<<24;
#else
	smp->common.TransactionID = (++g_transId) & 0xffffffff;
#endif
	smp->common.AttributeID = attr;
	smp->common.AttributeModifier = modifier;
	smp->M_Key = g_mkey;
	// rest of fields should be ignored for a Get, zero'ed above
	STL_BSWAP_SMP_HEADER(smp);
#ifdef IB_DEBUG
	DBGPRINT("Sending STL MAD:\n");
	DumpMad(smp);
#endif

	ASSERT(lid);
    recv_size = sizeof(*smp);
	fstatus = omgt_send_recv_mad_no_alloc(port, 
										 (uint8_t *)smp, sizeof(*smp), 
										 &addr,
										 (uint8_t *)smp, &recv_size,
										 RESP_WAIT_TIME, 
										 MAD_ATTEMPTS-1);
#ifdef IB_DEBUG
	if (fstatus == FSUCCESS) {
		DBGPRINT("Received STL MAD:\n");
		StlDumpMad(smp);
	}
#endif
	STL_BSWAP_SMP_HEADER(smp);
	//DBGPRINT("send_recv_mad fstatus=%s\n", iba_fstatus_msg(fstatus));
	return fstatus;
}
/* Get NodeDescription from SMA at lid
 * Retry as needed
 */
FSTATUS SmaGetNodeDesc(struct omgt_port *port, 
					   NodeData *nodep, 
					   uint32_t lid, 
					   STL_NODE_DESCRIPTION *pNodeDesc)
{
	STL_SMP smp;
	FSTATUS fstatus;

	MemoryClear(&smp, sizeof(smp));
	// rest of fields should be ignored for a Get, zero'ed above

	DBGPRINT("Sending SMA Get(NodeDesc) to LID 0x%x Node 0x%016"PRIx64"\n",
				lid,
				nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);

	fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, STL_MCLASS_ATTRIB_ID_NODE_DESCRIPTION, 0, &smp);

	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			*pNodeDesc = *(STL_NODE_DESCRIPTION*)stl_get_smp_data(&smp);
			BSWAP_STL_NODE_DESCRIPTION(pNodeDesc);
		}
	}
	return fstatus;
}

/* Get NodeInfo from SMA at lid
 * Retry as needed
 */
FSTATUS SmaGetNodeInfo(struct omgt_port *port, 
					   NodeData *nodep, 
					   uint32_t lid, 
					   STL_NODE_INFO *pNodeInfo)
{
	STL_SMP smp;
	FSTATUS fstatus;

	MemoryClear(&smp, sizeof(smp));
	// rest of fields should be ignored for a Get, zero'ed above

	DBGPRINT("Sending SMA Get(NodeInfo) to LID 0x%x Node 0x%016"PRIx64"\n",
				lid,
				nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);
	fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, STL_MCLASS_ATTRIB_ID_NODE_INFO, 0, &smp);
	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			*pNodeInfo = *(STL_NODE_INFO*)stl_get_smp_data(&smp);
			BSWAP_STL_NODE_INFO(pNodeInfo);
		}
	}
	return fstatus;
}

/* Get SwitchInfo from SMA at lid
 * Retry as needed
 */
FSTATUS SmaGetSwitchInfo(struct omgt_port *port, 
						 NodeData *nodep, 
						 uint32_t lid, 
						 STL_SWITCH_INFO *pSwitchInfo)
{
	STL_SMP smp;
	FSTATUS fstatus;

	MemoryClear(&smp, sizeof(smp));
	// rest of fields should be ignored for a Get, zero'ed above

	DBGPRINT("Sending SMA Get(SwitchInfo) to LID 0x%x Node 0x%016"PRIx64"\n",
				lid,
				nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);
	fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, STL_MCLASS_ATTRIB_ID_SWITCH_INFO, 0, &smp);
	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			*pSwitchInfo = *(STL_SWITCH_INFO*)stl_get_smp_data(&smp);
			BSWAP_STL_SWITCH_INFO(pSwitchInfo);
		}
	}
	return fstatus;
}

/* Get PortInfo from SMA at lid
 * Retry as needed
 */
FSTATUS SmaGetPortInfo(struct omgt_port *port,
					   NodeData *nodep, 
					   uint32_t lid, 
					   uint8_t portNum, 
					   STL_PORT_INFO *pPortInfo)
{
	STL_SMP smp;
	FSTATUS fstatus;
	uint32 amod = 0x01000000 | portNum;

	MemoryClear(&smp, sizeof(smp));
	// rest of fields should be ignored for a Get, zero'ed above

	DBGPRINT("Sending SMA Get(PortInfo %u) to LID 0x%x Node 0x%016"PRIx64"\n",
				portNum, lid,
				nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);
	fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, STL_MCLASS_ATTRIB_ID_PORT_INFO, amod, &smp);
	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			*pPortInfo = *(STL_PORT_INFO*)stl_get_smp_data(&smp);
			BSWAP_STL_PORT_INFO(pPortInfo);
		}
	}
	return fstatus;
}

/* Get CableInfo from SMA at lid, starting at address addr & length len
 * Retry as needed
 */

FSTATUS SmaGetCableInfo(struct omgt_port *port,
						NodeData *nodep, 
						uint32_t lid, 
						uint8_t portnum,
						uint16_t addr, 
						uint8_t len,
						uint8_t *data)
{
	STL_SMP smp;
	FSTATUS fstatus;
	uint32_t amod = (addr & 0x07ff)<<19 | (len & 0x3f)<<13 | (portnum & 0xff);
	STL_CABLE_INFO *pCableInfo;

	MemoryClear(&smp, sizeof(smp));

	DBGPRINT("Sending SMA Get(CableInfo %u) to LID 0x%x Node 0x%016"PRIx64"\n",
				portnum, lid, nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);
	fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, STL_MCLASS_ATTRIB_ID_CABLE_INFO, amod, &smp);
	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			pCableInfo = (STL_CABLE_INFO*)stl_get_smp_data(&smp);
			BSWAP_STL_CABLE_INFO(pCableInfo);
			memcpy(data, pCableInfo->Data, len+1);
		}
	}
	return fstatus;
}

/* Get Partition Table from SMA at lid
 * Retry as needed
 */
FSTATUS SmaGetPartTable(struct omgt_port *port,
						NodeData *nodep, 
						uint32_t lid, 
						uint8_t portNum, 
						uint16 block, 
						STL_PARTITION_TABLE *pPartTable)
{
	STL_SMP smp;
	FSTATUS fstatus;
	uint32 attrmod = (1<<24) | (portNum<<16) | block;

	MemoryClear(&smp, sizeof(smp));
	// rest of fields should be ignored for a Get, zero'ed above

	DBGPRINT("Sending SMA Get(P_KeyTable %u %u) to LID 0x%x Node 0x%016"PRIx64"\n",
				portNum, block, lid,
				nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);
	fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, STL_MCLASS_ATTRIB_ID_PART_TABLE, attrmod, &smp);
	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			*pPartTable = *(STL_PARTITION_TABLE*)stl_get_smp_data(&smp);
			BSWAP_STL_PARTITION_TABLE(pPartTable);
		}
	}
	return fstatus;
}

/* Get VLArb Table from SMA at lid
 * Retry as needed
 # Part has changed in STL, due to the table changes
 */
FSTATUS SmaGetVLArbTable(struct omgt_port *port,
						 NodeData *nodep, 
						 uint32_t lid, 
						 uint8_t portNum, 
						 uint8 part, 
						 STL_VLARB_TABLE *pVLArbTable)
{
	STL_SMP smp;
	FSTATUS fstatus;
	uint32 attrmod = (1<<24) | (part<<16) | portNum;

	MemoryClear(&smp, sizeof(smp));
	// rest of fields should be ignored for a Get, zero'ed above

	DBGPRINT("Sending SMA Get(VLArb %u %u) to LID 0x%x Node 0x%016"PRIx64"\n",
				part, portNum, lid,
				nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);
	fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, STL_MCLASS_ATTRIB_ID_VL_ARBITRATION, attrmod, &smp);
	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			*pVLArbTable = *(STL_VLARB_TABLE*)stl_get_smp_data(&smp);
			BSWAP_STL_VLARB_TABLE(pVLArbTable, part);
		}
	}
	return fstatus;
}

/* Get SL2SC Mapping Table from SMA at lid
 * Retry as needed
 */
FSTATUS SmaGetSLSCMappingTable(struct omgt_port *port,
							   NodeData *nodep, 
							   uint32_t lid, 
							   STL_SLSCMAP *pSLSCMap)
{
	STL_SMP smp;
	FSTATUS fstatus;
	uint32 attrmod = 0;

	MemoryClear(&smp, sizeof(smp));
	// rest of fields should be ignored for a Get, zero'ed above

	DBGPRINT("Sending SMA Get(SLSCMap) to LID 0x%x Node 0x%016"PRIx64"\n",
				lid, nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);
	fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, STL_MCLASS_ATTRIB_ID_SL_SC_MAPPING_TABLE, attrmod, &smp);
	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			*pSLSCMap = *(STL_SLSCMAP*)stl_get_smp_data(&smp);
			BSWAP_STL_SLSCMAP(pSLSCMap);
		}
	}
	return fstatus;
}

/* Get SC2SL Mapping Table from SMA at lid
 * Retry as needed
 */
FSTATUS SmaGetSCSLMappingTable(struct omgt_port *port,
							   NodeData *nodep, 
							   uint32_t lid, 
							   STL_SCSLMAP *pSCSLMap)
{
	STL_SMP smp;
	FSTATUS fstatus;
	uint32 attrmod = 0;

	MemoryClear(&smp, sizeof(smp));
	// rest of fields should be ignored for a Get, zero'ed above

	DBGPRINT("Sending SMA Get(SCSLMap) to LID 0x%x Node 0x%016"PRIx64"\n",
				lid, nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);
	fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, STL_MCLASS_ATTRIB_ID_SC_SL_MAPPING_TABLE, attrmod, &smp);
	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			*pSCSLMap = *(STL_SCSLMAP*)stl_get_smp_data(&smp);
			BSWAP_STL_SCSLMAP(pSCSLMap);
		}
	}
	return fstatus;
}

/* Get SC2SC Mapping Table from SMA at lid
 * Retry as needed
 */
FSTATUS SmaGetSCSCMappingTable(struct omgt_port *port,
							   NodeData *nodep, 
							   uint32_t lid, 
							   uint8_t in_port, 
							   uint8_t out_port, 
							   STL_SCSCMAP *pSCSCMap)
{
	STL_SMP smp;
	FSTATUS fstatus;
	uint32 attrmod = (1 << 24) | (in_port<<8) | out_port;

	MemoryClear(&smp, sizeof(smp));
	// rest of fields should be ignored for a Get, zero'ed above

	DBGPRINT("Sending SMA Get(SCSCMap %u %u) to LID 0x%x Node 0x%016"PRIx64"\n",
				in_port, out_port, lid,
				nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);
	fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, STL_MCLASS_ATTRIB_ID_SC_SC_MAPPING_TABLE, attrmod, &smp);
	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			*pSCSCMap = *(STL_SCSCMAP*)stl_get_smp_data(&smp);
			BSWAP_STL_SCSCMAP(pSCSCMap);
		}
	}
	return fstatus;
}

FSTATUS SmaGetSCVLMappingTable(struct omgt_port *port,
							   NodeData *nodep,
							   uint32_t lid,
							   uint8_t port_num,
							   STL_SCVLMAP *pSCVLMap,
							   uint16_t attr)
{
	STL_SMP smp;
	FSTATUS fstatus;
	uint32 attrmod = (1<<24) | port_num;

	MemoryClear(&smp, sizeof(smp));

	DBGPRINT("Sending SMA Get(SCVLMap %u) to LID 0x%x Node 0x%016"PRIx64"\n",
			 port_num, lid,
			 nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
			 STL_NODE_DESCRIPTION_ARRAY_SIZE,
			 (char*)nodep->NodeDesc.NodeString);
	fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, attr, attrmod, &smp);
	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			*pSCVLMap = *(STL_SCVLMAP*)stl_get_smp_data(&smp);
			BSWAP_STL_SCVLMAP(pSCVLMap);
		}
	}
	return fstatus;  
}

/* Get Buffer Control Table from SMA at lid
 * Retry as needed
 * pBCT must be large enough to hold all BufferControlTables
 */
FSTATUS SmaGetBufferControlTable(struct omgt_port *port,
							   NodeData *nodep,
							   uint32_t lid,
							   uint8_t startPort,
							   uint8_t endPort,
							   STL_BUFFER_CONTROL_TABLE pBCT[])
{
	STL_SMP smp;
	FSTATUS fstatus = FERROR;
    uint8_t maxcount = STL_NUM_BFRCTLTAB_BLOCKS_PER_LID_SMP;
	uint8_t block;

	MemoryClear(&smp, sizeof(smp));

	DBGPRINT("Sending SMA Get(BufferControlTable %u %u) to LID 0x%x Node 0x%016"PRIx64"\n",
				startPort, endPort, lid,
				nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);

	for (block = startPort; block <= endPort; block += maxcount) {
		uint8_t n = MIN(maxcount, (endPort - block)+1);
		uint32_t am = (n << 24) | block;
		fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, STL_MCLASS_ATTRIB_ID_BUFFER_CONTROL_TABLE,
										am, &smp);
		if (FSUCCESS == fstatus) {
			if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
				DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
				fstatus = FERROR;
			} else {
				int numPorts = (smp.common.AttributeModifier >> 24) & 0xff;
				int i;

				uint8_t * data = stl_get_smp_data(&smp);
				STL_BUFFER_CONTROL_TABLE *table = (STL_BUFFER_CONTROL_TABLE *)data;

				for (i = 0; i < numPorts; i++) {
					BSWAP_STL_BUFFER_CONTROL_TABLE(table);
					memcpy(&pBCT[block-startPort+i], table, sizeof(STL_BUFFER_CONTROL_TABLE));
					// Handle the dissimilar sizes of Buffer Table and 8-byte pad alignment
					data += STL_BFRCTRLTAB_PAD_SIZE;
					table = (STL_BUFFER_CONTROL_TABLE *)(data);
				}
			}
		}
	}

	return fstatus;
}

/* Get Linear FDB Table from SMA at lid
 * Retry as needed
 */
FSTATUS SmaGetLinearFDBTable(struct omgt_port *port,
							 NodeData *nodep, 
							 uint32_t lid, 
							 uint16 block, 
							 STL_LINEAR_FORWARDING_TABLE *pFDB)
{
	STL_SMP smp;
	FSTATUS fstatus;
	uint32_t  modifier;

	MemoryClear(&smp, sizeof(smp));
	// rest of fields should be ignored for a Get, zero'ed above

	DBGPRINT("Sending SMA Get(LFT %u) to LID 0x%x Node 0x%016"PRIx64"\n",
				block, lid,
				nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);
	modifier = 0x01000000 + (uint32_t)block;
	fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, STL_MCLASS_ATTRIB_ID_LINEAR_FWD_TABLE, modifier, &smp);
	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			*pFDB = *(STL_LINEAR_FORWARDING_TABLE*)stl_get_smp_data(&smp);
			BSWAP_STL_LINEAR_FORWARDING_TABLE(pFDB);
		}
	}
	return fstatus;
}

/* Get Multicast FDB Table from SMA at lid
 * Retry as needed
 */
FSTATUS SmaGetMulticastFDBTable(struct omgt_port *port,
								NodeData *nodep, 
								uint32_t lid, 
								uint32 block, 
								uint8 position, 
								STL_MULTICAST_FORWARDING_TABLE *pFDB)
{
	STL_SMP smp;
	FSTATUS fstatus;

	MemoryClear(&smp, sizeof(smp));
	// rest of fields should be ignored for a Get, zero'ed above

	DBGPRINT("Sending SMA Get(MFT %u %u) to LID 0x%x Node 0x%016"PRIx64"\n",
				block, position, lid,
				nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);
	//@TODO: Enable multi-block requests from just a single block request
	fstatus = stl_sma_send_recv_mad(port, 
									lid, 
									MMTHD_GET, 
									STL_MCLASS_ATTRIB_ID_MCAST_FWD_TABLE, 
									(0x1<<24) | (0x3 & position)<<22 | (block & 0xfffff), 
									&smp);
	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			*pFDB = *(STL_MULTICAST_FORWARDING_TABLE*)stl_get_smp_data(&smp);
			BSWAP_STL_MULTICAST_FORWARDING_TABLE(pFDB);
		}
	}
	return fstatus;
}

/* Get PortGroup Table from SMA at lid
 * Retry as needed
 */
FSTATUS SmaGetPortGroupTable(struct omgt_port *port,
							 NodeData *nodep,
							 uint32_t lid,
							 uint16 block,
							 STL_PORT_GROUP_TABLE *pPGT)
{
	STL_SMP smp;
	FSTATUS fstatus;
	uint32_t  modifier;

	MemoryClear(&smp, sizeof(smp));
	// rest of fields should be ignored for a Get, zero'ed above

	DBGPRINT("Sending SMA Get(PortGroup %u) to LID 0x%x Node 0x%016"PRIx64"\n",
				block, lid,
				nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);
	modifier = 0x01000000 + (uint32_t)block;
	fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, STL_MCLASS_ATTRIB_ID_PORT_GROUP_TABLE, modifier, &smp);
	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			*pPGT = *(STL_PORT_GROUP_TABLE*)stl_get_smp_data(&smp);
			BSWAP_STL_PORT_GROUP_TABLE(pPGT);
		}
	}
	return fstatus;
}

/* Get PortGroup FDB Table from SMA at lid
 * Retry as needed
 */
FSTATUS SmaGetPortGroupFDBTable(struct omgt_port *port,
							 NodeData *nodep,
							 uint32_t lid,
							 uint16 block,
							 STL_PORT_GROUP_FORWARDING_TABLE *pFDB)
{
	STL_SMP smp;
	FSTATUS fstatus;
	uint32_t  modifier;

	MemoryClear(&smp, sizeof(smp));
	// rest of fields should be ignored for a Get, zero'ed above

	DBGPRINT("Sending SMA Get(PortGroupFDB %u) to LID 0x%x Node 0x%016"PRIx64"\n",
				block, lid,
				nodep->NodeInfo.NodeGUID);
	DBGPRINT("    Name: %.*s\n",
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);
	modifier = 0x01000000 + (uint32_t)block;
	fstatus = stl_sma_send_recv_mad(port, lid, MMTHD_GET, STL_MCLASS_ATTRIB_ID_PORT_GROUP_FWD_TABLE, modifier, &smp);
	if (FSUCCESS == fstatus) {
		if (smp.common.u.DR.s.Status != MAD_STATUS_SUCCESS) {
			DBGPRINT("SMA response with bad status: 0x%x\n", smp.common.u.DR.s.Status);
			fstatus = FERROR;
		} else {
			*pFDB = *(STL_PORT_GROUP_FORWARDING_TABLE*)stl_get_smp_data(&smp);
			BSWAP_STL_PORT_GROUP_FORWARDING_TABLE(pFDB);
		}
	}
	return fstatus;
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
										 MAD_ATTEMPTS-1);
	
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

	DBGPRINT("Sending PM Get(ClassPortInfo) to LID 0x%04x Node 0x%016"PRIx64"\n",
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
							STL_PortStatusData_t *pPortStatus)
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
	BSWAP_STL_PORT_STATUS_RSP((STL_PORT_STATUS_RSP*)resp.PerfData);
	*pPortStatus = *(STL_PortStatusData_t*)resp.PerfData;
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

	ASSERT(pathp->DLID);
    recv_size = sizeof(*mad);
	fstatus = omgt_send_recv_mad_no_alloc(port, 
										 (uint8_t *)mad, sizeof(*mad),
										 &addr,
										 (uint8_t *)mad, &recv_size,
										 RESP_WAIT_TIME, 
										 MAD_ATTEMPTS-1);

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
