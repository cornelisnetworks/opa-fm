/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifdef IB_STACK_OPENIB


#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <assert.h>
#include <ctype.h>
#include <string.h>
#include <errno.h>

#include <infiniband/umad_types.h>
#include <infiniband/verbs.h>
#include <infiniband/umad.h>
#include <infiniband/umad_sa.h>
#include <infiniband/umad_sm.h>

#define OPAMGT_PRIVATE 1

// Includes / Defines from IBACCESS
#include "ibt.h"
#include "opamgt_priv.h"
#include "ib_utils_openib.h"
#include "stl_pa_priv.h"
#include "ib_mad.h"
#include "opamgt_dump_mad.h"
#include "stl_sd.h"
#include "stl_pm.h"
#include "opamgt_pa_priv.h"
#include "opamgt_sa_priv.h"

/************************************** 
 * Defines and global variables 
 **************************************/

#define IB_SA_DATA_OFFS         56

#define MAX_PORTS_LIST          300000  // Max number of ports in port list

#define OPAMGT_MAD_SIZE      256
#define OPAMGT_DEF_RETRIES   3
#define MAD_STATUS_PA_MINIMUM   0x0A00

// from ib_sa.h
#define	PR_COMPONENTMASK_SRV_ID	0x0000000000000001ull
#define	PR_COMPONENTMASK_DGID	0x0000000000000004ull
#define	PR_COMPONENTMASK_SGID	0x0000000000000008ull

//from ib_sa_records.h
#define IB_MULTIPATH_RECORD_COMP_NUMBPATH	0x00000040

static uint8_t ib_truescale_oui[] = {OUI_TRUESCALE_0, OUI_TRUESCALE_1, OUI_TRUESCALE_2};

static const char* const pa_query_input_type_text[] = 
{
    "InputTypeNoInput",
};

static const char* const pa_query_result_type_text[] = 
{
    "OutputTypePaTableRecord",        /* PA_TABLE_PACKET_RESULTS complete PA MultiPacketRespRecords */
};

static const char* const pa_sa_status_text[] =
{
    "Success",  // not used by code below
	"PA: Insufficient Resources",             //  0x0100 NO_RESOURCES
	"PA: Invalid Request",                    //  0x0200 REQ_INVALID
	"PA: No Records",                         //  0x0300 NO_RECORDS
	"PA: Too Many Records",                   //  0x0400 TOO_MANY_RECORDS
	"PA: Invalid GID in Request",             //  0x0500 REQ_INVALID_GID
	"PA: Insufficient Components in Request", //  0x0600 REQ_INSUFFICIENT_COMPONENT
};
static const char* const pa_status_text[] =
{
    "Success",  // not used by code below
    "PA: PM Engine Unavailable",			//0x0A00 - Engine unavailable
    "PA: Group Not Found",  				//0x0B00 - No such group
    "PA: Port Not Found", 					//0x0C00 - Port not found
    "PA: Virtual Fabric Not Found", 		//0x0D00 - VF not found
    "PA: Invalid Parameter",  				//0x0E00 - Invalid parameter
    "PA: Image Not Found",  				//0x0F00 - Image not found
    "PA: No Data",  					//0x1000 - no data skipped port
    "PA: Bad Data",  					//0x1100 - query fail or clear fail
};

/**************************************** 
 * Prototypes
 ****************************************/


/******************************************************** 
 * Local file functions
 *******************************************************/

/** 
 *  Send a PA query and get the result.
 * 
 * @param port              The port from which we access the fabric.
 * @param method            PA method identifier.
 * @param attr_id           PA attribute identifier
 * @param attr_mod          PA attribute modifier
 * @param snd_data          Outbound request data.
 * @param snd_data_len      Outbound request data length.
 * @param rcv_buf_len       Max rcv buffer length (required for OFED driver)
 * @param rsp_mad           Response MAD packet. The caller should free it after use.
 * @param query_result      Query return result (pointer to pointer to the query result). Allocated
 *                          here if successful and the caller must free it.
 *
 * @return 
 *   FSUCCESS - Query successful
 *     FERROR - Error
 */
static FSTATUS
pa_query_common(
	struct omgt_port      *port,
	uint8_t                method,
	uint32_t               attr_id,
	uint32_t               attr_mod,
	uint8_t               *snd_data,
	size_t                 snd_data_len,
	size_t                *rcv_buf_len,
	SA_MAD               **rsp_mad,
	PQUERY_RESULT_VALUES  *query_result
	)
{
	FSTATUS    fstatus = FSUCCESS;
	uint32_t   rec_sz = 0;
	uint32_t   rec_cnt = 0;
	uint32_t   mem_size;
	SA_MAD	   *send_mad = (SA_MAD *)snd_data;
	SA_MAD     *rcv_mad = NULL;
	struct omgt_mad_addr addr = {0};
	MAD_STATUS madStatus = {0};
	static uint32_t mad_tid = 1;

	// do some common stuff for each command / query.
	DBG_ENTER_FUNC(port);

	/* If port is In-Band, set up addr */
	if (!port->is_oob_enabled) {
		uint8_t port_state;
		(void)omgt_port_get_port_state(port, &port_state);
		if (port_state != PortStateActive) {
			OMGT_OUTPUT_ERROR(port, "Local port not Active!\n");
			return FINVALID_STATE;
		}
		if ((port->pa_service_state != OMGT_SERVICE_STATE_OPERATIONAL)
			&& (omgt_pa_service_connect(port) != OMGT_SERVICE_STATE_OPERATIONAL)) {

			OMGT_OUTPUT_ERROR(port, "Query PA failed: PA Service Not Operational: %s (%d)\n",
				omgt_service_state_totext(port->pa_service_state), port->pa_service_state);
			return FUNAVAILABLE;
		}
		addr.lid = port->primary_pm_lid;
		addr.sl = port->primary_pm_sl;
		addr.qpn = 1;
		addr.qkey = QP1_WELL_KNOWN_Q_KEY;
		addr.pkey = OMGT_DEFAULT_PKEY;
		if (omgt_find_pkey(port, OMGT_DEFAULT_PKEY) < 0) {
			OMGT_OUTPUT_ERROR(port, "Query PA failed: requires full management node. Status:(%u)\n", FPROTECTION);
			return FPROTECTION;
		}
	}

	OMGT_DBGPRINT(port, "Request MAD method: 0x%x\n", method);
	OMGT_DBGPRINT(port, "\taid: 0x%x\n", attr_id);
	OMGT_DBGPRINT(port, "\tamod: 0x%x\n", attr_mod);

	// default the parameters to failed state.
	*query_result = NULL;
	*rsp_mad = NULL;

	/* Setup MAD Header */
	MAD_SET_VERSION_INFO(send_mad, STL_BASE_VERSION, MCLASS_VFI_PM, STL_PA_CLASS_VERSION);
	MAD_SET_METHOD_TYPE(send_mad, method);
	MAD_SET_ATTRIB_ID(send_mad, attr_id);
	MAD_SET_ATTRIB_MOD(send_mad, attr_mod); /* should be zero */
	MAD_SET_TRANSACTION_ID(send_mad, mad_tid++);

	BSWAP_SA_HDR(&send_mad->SaHdr);
	BSWAP_MAD_HEADER((MAD *)&send_mad->common);

	/* Add OUI to MAD after BSWAP */
	struct umad_vendor_packet *pkt = (struct umad_vendor_packet *)send_mad;
	memcpy(&pkt->oui, ib_truescale_oui, 3);

	// submit RMPP MAD request
	fstatus = omgt_send_recv_mad_alloc(port, snd_data, (size_t)snd_data_len, &addr,
		(uint8_t **)rsp_mad, rcv_buf_len, port->ms_timeout, port->retry_count);

	if (fstatus != FSUCCESS) {
		if (fstatus == FPROTECTION) {
			// PKEY lookup error.
			OMGT_OUTPUT_ERROR(port, "Query Failed: requires full management node.\n");
		} else {
			OMGT_DBGPRINT(port, "Query Failed: %u.\n", (unsigned int)fstatus);
			port->pa_service_state = OMGT_SERVICE_STATE_DOWN;
		}
		goto done;
	}

	if (*rcv_buf_len < PA_REQ_HEADER_SIZE) {
		OMGT_DBGPRINT(port, "Query PA: Failed to receive packet\n");
		fstatus = FNOT_FOUND;
		goto done;
	}

	rcv_mad = *rsp_mad;
	BSWAP_MAD_HEADER((MAD *)&rcv_mad->common);
	BSWAP_SA_HDR(&rcv_mad->SaHdr);
	MAD_GET_STATUS(rcv_mad, &madStatus);

	// dump of PA header for debug
	OMGT_DBGPRINT(port, " PA Header\n");
	OMGT_DBGPRINT(port, " length %zu (0x%zx) vs Header length %d\n",
		*rcv_buf_len, *rcv_buf_len, (int)PA_REQ_HEADER_SIZE);
	//OMGT_DBGPRINT(port, " SmKey (0x%016"PRIx64")\n", rcv_mad->SaHdr.SmKey);
	OMGT_DBGPRINT(port, " AttributeOffset %u (0x%x) : in bytes: %u\n",
		rcv_mad->SaHdr.AttributeOffset, rcv_mad->SaHdr.AttributeOffset,
		(rcv_mad->SaHdr.AttributeOffset * 8));
	OMGT_DBGPRINT(port, " Reserved (0x%x)\n", rcv_mad->SaHdr.Reserved);
	//OMGT_DBGPRINT(port, " ComponetMask (0x%016"PRIx64")\n", rcv_mad->SaHdr.ComponentMask);

	rec_sz = rcv_mad->SaHdr.AttributeOffset * 8;
	if (rcv_mad->common.mr.AsReg8 == STL_PA_CMD_GET_RESP) {
		/* Attribute Offset is 0 on GetResp() as there is only one record */
		rec_sz = *rcv_buf_len - PA_REQ_HEADER_SIZE;
		rec_cnt = 1;
	} else if (rcv_mad->SaHdr.AttributeOffset) {
		rec_cnt = (int)((*rcv_buf_len - PA_REQ_HEADER_SIZE) /
			(rcv_mad->SaHdr.AttributeOffset * 8));
	} else {
		rec_cnt = 0;
	}
	OMGT_DBGPRINT(port, "Record count is %d\n", rec_cnt);

	// If the command was not successful (mad status is not 0)
	// Change overall return status to failed.
	// Fill in the result pointer so the caller can evaluate the return mad status.
	port->pa_mad_status = madStatus.AsReg16;
	if (madStatus.AsReg16 != 0) {
		OMGT_DBGPRINT(port, "Query PA failed: Mad status is 0x%x: %s\n",
			madStatus.AsReg16, iba_pa_mad_status_msg(port));

		switch (madStatus.AsReg16) {
		case STL_MAD_STATUS_STL_PA_NO_GROUP:
		case STL_MAD_STATUS_STL_PA_NO_PORT:
		case STL_MAD_STATUS_STL_PA_NO_VF:
		case STL_MAD_STATUS_STL_PA_NO_IMAGE:
		case STL_MAD_STATUS_STL_PA_NO_DATA:
			fstatus = FNOT_FOUND;
			break;
		case STL_MAD_STATUS_STL_PA_INVALID_PARAMETER:
			fstatus = FINVALID_PARAMETER;
			break;
		case STL_MAD_STATUS_STL_PA_UNAVAILABLE:
			fstatus = FINVALID_STATE;
			break;
		case STL_MAD_STATUS_STL_PA_BAD_DATA:
		default:
			fstatus = FERROR;
			break;
		}
	}

	mem_size  = rec_sz * rec_cnt;
	mem_size += sizeof(uint32_t);
	mem_size += sizeof(QUERY_RESULT_VALUES);

	// resultDataSize should be 0 when status is not successful and no data is returned
	*query_result = MemoryAllocate2AndClear(mem_size, IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG);
	if (*query_result == NULL) {
		OMGT_OUTPUT_ERROR(port, "error allocating query result buffer\n");
		fstatus = FINSUFFICIENT_MEMORY;
		goto done;
	}
	(*query_result)->Status = fstatus;
	(*query_result)->MadStatus = port->pa_mad_status;
	(*query_result)->ResultDataSize = rec_sz * rec_cnt;
	*((uint32_t *)((*query_result)->QueryResult)) = rec_cnt;

done:
	if (fstatus != FSUCCESS) {
		if (*rsp_mad != NULL) {
			free(*rsp_mad);
			*rsp_mad = NULL;
		}
	}

	DBG_EXIT_FUNC(port);
	return (fstatus);
}   // End of pa_query_common()

const char* 
iba_pa_query_input_type_msg(
    QUERY_INPUT_TYPE    code
    )
{
    if (code < 0 || code >= (int)(sizeof(pa_query_input_type_text)/sizeof(char*)))
        return "Unknown PA Query Input Type";
    else
        return pa_query_input_type_text[code];
}

const char* 
iba_pa_query_result_type_msg(
    QUERY_RESULT_TYPE   code
    )
{
    if (code < 0 || code >= (int)(sizeof(pa_query_result_type_text)/sizeof(char*)))
        return "Unknown PA Query Result Type";
    else
        return pa_query_result_type_text[code];
}


/******************************************************* 
 * Public functions
 *******************************************************/
/**
 *   Enable debugging output in the opamgt library.
 *
 * @param   port    The local port to operate on.
 *  
 * @return 
 *   None
 */
void 
set_opapaquery_debug (
    IN struct omgt_port  *port
    )
{
    port->pa_verbose = 1;

    // Should we pass along the debug info
    //_openib_debug_ = 1;     // enable general debugging

    return;
}

/**
 *  Get the classportinfo from the given port.
 *
 * @param port                  Local port to operate on.  
 *  
 * @return 
 *   Valid pointer  -   A pointer to the ClassPortInfo. The caller must free it after use.
 *   NULL           -   Error.
 */
OMGT_STATUS_T
omgt_pa_get_classportinfo(
    IN struct omgt_port  *port,
	STL_CLASS_PORT_INFO **cpi
    )
{
    FSTATUS                 fstatus = FSUCCESS;
    QUERY_RESULT_VALUES     *query_result = NULL;
    STL_CLASS_PORT_INFO     *response = NULL;
    SA_MAD                  *rsp_mad = NULL;
    size_t                  rcv_buf_len = 0;
    uint8_t                 request_data[PA_REQ_HEADER_SIZE] = {0};

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return FERROR;

	// submit request
	fstatus = pa_query_common(port, STL_PA_CMD_GET, STL_PA_ATTRID_GET_CLASSPORTINFO, 0,
		request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
	if (fstatus != FSUCCESS) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
		goto done;
	} else if (rsp_mad->SaHdr.AttributeOffset) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, unexpected multiple MAD response\n");
		fstatus = FERROR;
		goto done;
	} else {
		if (port->pa_verbose) {
			OMGT_DBGPRINT(port, "Completed request: OK\n");
		}
	}

	response = MemoryAllocate2AndClear(sizeof(STL_CLASS_PORT_INFO), IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG);
	if (response == NULL) {
		OMGT_OUTPUT_ERROR(port, "error allocating response buffer\n");
		fstatus = FINSUFFICIENT_RESOURCES;
		goto done;
	}
	memcpy((uint8 *)response, rsp_mad->Data, min(sizeof(STL_CLASS_PORT_INFO), rcv_buf_len - IB_SA_DATA_OFFS));

	// translate the data.
	BSWAP_STL_CLASS_PORT_INFO(response);
done:
	omgt_free_query_result_buffer(query_result);
	if (rsp_mad)
		free(rsp_mad);
	*cpi = response;
	DBG_EXIT_FUNC(port);
	return fstatus;

}   // End of omgt_pa_get_classportinfo()

/**
 *  Get port statistics (counters)
 *
 * @param port                  Local port to operate on. 
 * @param node_lid              Remote node LID.
 * @param port_num              Remote port number. 
 * @param port_counters         Pointer to port counters to fill.
 * @param delta_flag            1 for delta counters, 0 for raw image counters.
 * @param user_ctrs_flag        1 for running counters, 0 for image counters. (delta must be 0)
 * @param image_id              Pointer to image ID of port counters to get. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the counters data. The caller must free it.
 *     NULL         - Error
 */
STL_PORT_COUNTERS_DATA *
iba_pa_single_mad_port_counters_response_query(
    IN struct omgt_port  *port,
    IN STL_LID           node_lid,
    IN uint8_t           port_number,
    IN uint32_t          delta_flag,
    IN uint32_t          user_cntrs_flag,
    IN STL_PA_IMAGE_ID_DATA    *image_id
    )
{
    FSTATUS                 fstatus = FSUCCESS;
    QUERY_RESULT_VALUES     *query_result = NULL;
    STL_PORT_COUNTERS_DATA  *response = NULL, *p;
    SA_MAD                  *rsp_mad = NULL;
    size_t                  rcv_buf_len = 0;
    uint8_t                 request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PORT_COUNTERS_DATA)] = {0};

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return response;

	// Build Request
    p = (STL_PORT_COUNTERS_DATA *)(((SA_MAD *)request_data)->Data);
    p->nodeLid = node_lid;
    p->portNumber = port_number;
    p->flags = (delta_flag ? STL_PA_PC_FLAG_DELTA : 0) |
			   (user_cntrs_flag ? STL_PA_PC_FLAG_USER_COUNTERS : 0);

    p->imageId.imageNumber = image_id->imageNumber;
    p->imageId.imageOffset = image_id->imageOffset;
	p->imageId.imageTime.absoluteTime = image_id->imageTime.absoluteTime;
	memset(p->reserved, 0, sizeof(p->reserved));
	memset(p->reserved2, 0, sizeof(p->reserved2));
	p->lq.s.reserved = 0;
    BSWAP_STL_PA_PORT_COUNTERS(p);

	// submit request
	fstatus = pa_query_common(port, STL_PA_CMD_GET, STL_PA_ATTRID_GET_PORT_CTRS, 0,
		request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
	if (fstatus != FSUCCESS) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
		goto done;
	} else if (rsp_mad->SaHdr.AttributeOffset) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, unexpected multiple MAD response\n");
		goto done;
	} else {
		if (port->pa_verbose) {
			OMGT_DBGPRINT(port, "Completed request: OK\n");
		}
	}

	response = MemoryAllocate2AndClear(sizeof(STL_PORT_COUNTERS_DATA), IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG);
	if (response == NULL) {
		OMGT_OUTPUT_ERROR(port, "error allocating response buffer\n");
		goto done;
	}
	memcpy((uint8 *)response, rsp_mad->Data, min(sizeof(STL_PORT_COUNTERS_DATA), rcv_buf_len - IB_SA_DATA_OFFS));

	// translate the data.
	BSWAP_STL_PA_PORT_COUNTERS(response);
done:
	omgt_free_query_result_buffer(query_result);
	if (rsp_mad)
		free(rsp_mad);

	DBG_EXIT_FUNC(port);
	return response;
}


/**
 *  Clear port statistics (counters)
 *
 * @param port                  Local Port to operate on. 
 * @param node_lid              Remote node LID.
 * @param port_num              Remote port number. 
 * @param select                Port's counters to clear. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the clear port counters data (containing the
 *                    info as which counters have been cleared).  The caller must free it.
 *     NULL         - Error
 */
STL_CLR_PORT_COUNTERS_DATA *
iba_pa_single_mad_clr_port_counters_response_query(
    IN struct omgt_port  *port,
    IN uint32_t          node_lid,
    IN uint8_t           port_number,
    IN uint32_t          select
    )
{
    FSTATUS                 fstatus = FSUCCESS;
    QUERY_RESULT_VALUES     *query_result = NULL;
    STL_CLR_PORT_COUNTERS_DATA  *response = NULL, *p;
    SA_MAD                  *rsp_mad = NULL;
    size_t                  rcv_buf_len = 0;
    uint8_t                 request_data[PA_REQ_HEADER_SIZE + sizeof(STL_CLR_PORT_COUNTERS_DATA)] = {0};

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return response;

    p = (STL_CLR_PORT_COUNTERS_DATA *)(((SA_MAD *)request_data)->Data);
    p->NodeLid = node_lid;
    p->PortNumber = port_number;
    p->CounterSelectMask.AsReg32 = select;
	p->CounterSelectMask.s.Reserved = 0;
	memset(p->Reserved, 0, sizeof(p->Reserved));
    BSWAP_STL_PA_CLR_PORT_COUNTERS(p);

    // submit request
	fstatus = pa_query_common(port, STL_PA_CMD_SET, STL_PA_ATTRID_CLR_PORT_CTRS, 0,
		request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
	if (fstatus != FSUCCESS) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
		goto done;
	} else if (rsp_mad->SaHdr.AttributeOffset) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, unexpected multiple MAD response\n");
		goto done;
	} else {
		if (port->pa_verbose) {
			OMGT_DBGPRINT(port, "Completed request: OK\n");
		}
	}

	response = MemoryAllocate2AndClear(sizeof(STL_CLR_PORT_COUNTERS_DATA), IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG);
	if (response == NULL) {
		OMGT_OUTPUT_ERROR(port, "error allocating response buffer\n");
		goto done;
	}
	memcpy((uint8 *)response, rsp_mad->Data, min(sizeof(STL_CLR_PORT_COUNTERS_DATA), rcv_buf_len - IB_SA_DATA_OFFS));

	// translate the data.
	BSWAP_STL_PA_CLR_PORT_COUNTERS(response);
done:
	omgt_free_query_result_buffer(query_result);
	if (rsp_mad)
		free(rsp_mad);

	DBG_EXIT_FUNC(port);
	return response;
}

/**
 *  Clear all ports, statistics (counters)
 *
 * @param port                  Local port to operate on. 
 * @param select                Port's counters to clear. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the clear all port counters data (containing the
 *                    info as which counters have been cleared).  The caller must free it.
 *     NULL         - Error
 */
STL_CLR_ALL_PORT_COUNTERS_DATA *
iba_pa_single_mad_clr_all_port_counters_response_query(
    IN struct omgt_port  *port,
    IN uint32_t          select
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    STL_CLR_ALL_PORT_COUNTERS_DATA  *response = NULL, *p;
    SA_MAD                      *rsp_mad = NULL;
    size_t                      rcv_buf_len = 0;
    uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_CLR_ALL_PORT_COUNTERS_DATA)] = {0};

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return response;

    p = (STL_CLR_ALL_PORT_COUNTERS_DATA *)(((SA_MAD *)request_data)->Data);
    p->CounterSelectMask.AsReg32 = select;
	p->CounterSelectMask.s.Reserved = 0;
	BSWAP_STL_PA_CLR_ALL_PORT_COUNTERS(p);

    // submit request
	fstatus = pa_query_common(port, STL_PA_CMD_SET, STL_PA_ATTRID_CLR_ALL_PORT_CTRS, 0,
		request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
	if (fstatus != FSUCCESS) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
		goto done;
	} else if (rsp_mad->SaHdr.AttributeOffset) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, unexpected multiple MAD response\n");
		goto done;
	} else {
		if (port->pa_verbose) {
			OMGT_DBGPRINT(port, "Completed request: OK\n");
		}
	}

	response = MemoryAllocate2AndClear(sizeof(STL_CLR_ALL_PORT_COUNTERS_DATA), IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG);
	if (response == NULL) {
		OMGT_OUTPUT_ERROR(port, "error allocating response buffer\n");
		goto done;
	}
	memcpy((uint8 *)response, rsp_mad->Data, min(sizeof(STL_CLR_ALL_PORT_COUNTERS_DATA), rcv_buf_len - IB_SA_DATA_OFFS));

	// translate the data.
	BSWAP_STL_PA_CLR_ALL_PORT_COUNTERS(response);
done:
	omgt_free_query_result_buffer(query_result);
	if (rsp_mad)
		free(rsp_mad);

	DBG_EXIT_FUNC(port);
	return response;
}

/**
 *  Get the PM config data.
 *
 * @param port                  Local port to operate on. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for pm config data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_PM_CFG_DATA *
iba_pa_single_mad_get_pm_config_response_query(
    IN struct omgt_port  *port
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    STL_PA_PM_CFG_DATA       	*response = NULL;
    SA_MAD                      *rsp_mad = NULL;
    size_t                      rcv_buf_len = 0;
    uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_PM_CFG_DATA)] = {0};

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return response;

	// submit request
	fstatus = pa_query_common(port, STL_PA_CMD_GET, STL_PA_ATTRID_GET_PM_CONFIG, 0,
		request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
	if (fstatus != FSUCCESS) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
		goto done;
	} else if (rsp_mad->SaHdr.AttributeOffset) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, unexpected multiple MAD response\n");
		goto done;
	} else {
		if (port->pa_verbose) {
			OMGT_DBGPRINT(port, "Completed request: OK\n");
		}
	}

	response = MemoryAllocate2AndClear(sizeof(STL_PA_PM_CFG_DATA), IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG);
	if (response == NULL) {
		OMGT_OUTPUT_ERROR(port, "error allocating response buffer\n");
		goto done;
	}
	memcpy((uint8 *)response, rsp_mad->Data, min(sizeof(STL_PA_PM_CFG_DATA), rcv_buf_len - IB_SA_DATA_OFFS));

	// translate the data.
	BSWAP_STL_PA_PM_CFG(response);
done:
	omgt_free_query_result_buffer(query_result);
	if (rsp_mad)
		free(rsp_mad);

	DBG_EXIT_FUNC(port);
	return response;
}

/**
 *  Free a sweep image.
 *
 * @param port                  Local port to operate on. 
 * @param image_id              ID of image to freeze. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the frozen image id data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_IMAGE_ID_DATA *
iba_pa_single_mad_freeze_image_response_query(
    IN struct omgt_port  *port,
    IN STL_PA_IMAGE_ID_DATA    *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    STL_PA_IMAGE_ID_DATA        *response = NULL, *p;
    SA_MAD                      *rsp_mad = NULL;
    size_t                      rcv_buf_len = 0;
    uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_IMAGE_ID_DATA)] = {0};

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return response;

    p = (STL_PA_IMAGE_ID_DATA *)(((SA_MAD *)request_data)->Data);
    p->imageNumber = hton64(image_id->imageNumber);
    p->imageOffset = hton32(image_id->imageOffset);
	p->imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);

	// submit request
	fstatus = pa_query_common(port, STL_PA_CMD_SET, STL_PA_ATTRID_FREEZE_IMAGE, 0,
		request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
	if (fstatus != FSUCCESS) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
		goto done;
	} else if (rsp_mad->SaHdr.AttributeOffset) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, unexpected multiple MAD response\n");
		goto done;
	} else {
		if (port->pa_verbose) {
			OMGT_DBGPRINT(port, "Completed request: OK\n");
		}
	}

	response = MemoryAllocate2AndClear(sizeof(STL_PA_IMAGE_ID_DATA), IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG);
	if (response == NULL) {
		OMGT_OUTPUT_ERROR(port, "error allocating response buffer\n");
		goto done;
	}
	memcpy((uint8 *)response, rsp_mad->Data, min(sizeof(STL_PA_IMAGE_ID_DATA), rcv_buf_len - IB_SA_DATA_OFFS));

	// translate the data.
	BSWAP_STL_PA_IMAGE_ID(response);
done:
	omgt_free_query_result_buffer(query_result);
	if (rsp_mad)
		free(rsp_mad);

	DBG_EXIT_FUNC(port);
	return response;
}

/**
 *  Release a sweep image.
 *
 * @param port                  Local port to operate on. 
 * @param image_id              Ponter to id of image to release. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the released image id data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_IMAGE_ID_DATA *
iba_pa_single_mad_release_image_response_query(
    IN struct omgt_port  *port,
    IN STL_PA_IMAGE_ID_DATA    *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    STL_PA_IMAGE_ID_DATA        *response = NULL, *p;
    SA_MAD                      *rsp_mad = NULL;
    size_t                      rcv_buf_len = 0;
    uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_IMAGE_ID_DATA)] = {0};

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return response;

    p = (STL_PA_IMAGE_ID_DATA *)(((SA_MAD *)request_data)->Data);
    p->imageNumber = hton64(image_id->imageNumber);
    p->imageOffset = hton32(image_id->imageOffset);
	p->imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);

	// submit request
	fstatus = pa_query_common(port, STL_PA_CMD_SET, STL_PA_ATTRID_RELEASE_IMAGE, 0,
		request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
	if (fstatus != FSUCCESS) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
		goto done;
	} else if (rsp_mad->SaHdr.AttributeOffset) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, unexpected multiple MAD response\n");
		goto done;
	} else {
		if (port->pa_verbose) {
			OMGT_DBGPRINT(port, "Completed request: OK\n");
		}
	}

	response = MemoryAllocate2AndClear(sizeof(STL_PA_IMAGE_ID_DATA), IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG);
	if (response == NULL) {
		OMGT_OUTPUT_ERROR(port, "error allocating response buffer\n");
		goto done;
	}
	memcpy((uint8 *)response, rsp_mad->Data, min(sizeof(STL_PA_IMAGE_ID_DATA), rcv_buf_len - IB_SA_DATA_OFFS));

	// translate the data.
	BSWAP_STL_PA_IMAGE_ID(response);
done:
	omgt_free_query_result_buffer(query_result);
	if (rsp_mad)
		free(rsp_mad);

	DBG_EXIT_FUNC(port);
	return response;
}

/**
 *  Renew a sweep image.
 *
 * @param port                  Local port to operate on. 
 * @param image_id              Ponter to id of image to renew. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the renewed image id data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_IMAGE_ID_DATA *
iba_pa_single_mad_renew_image_response_query(
    IN struct omgt_port  *port,
    IN STL_PA_IMAGE_ID_DATA    *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    STL_PA_IMAGE_ID_DATA        *response = NULL, *p;
    SA_MAD                      *rsp_mad = NULL;
    size_t                      rcv_buf_len = 0;
    uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_IMAGE_ID_DATA)] = {0};

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return response;

    p = (STL_PA_IMAGE_ID_DATA *)(((SA_MAD *)request_data)->Data);
    p->imageNumber = hton64(image_id->imageNumber);
    p->imageOffset = hton32(image_id->imageOffset);
	p->imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);

	// submit request
	fstatus = pa_query_common(port, STL_PA_CMD_SET, STL_PA_ATTRID_RENEW_IMAGE, 0,
		request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
	if (fstatus != FSUCCESS) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
		goto done;
	} else if (rsp_mad->SaHdr.AttributeOffset) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, unexpected multiple MAD response\n");
		goto done;
	} else {
		if (port->pa_verbose) {
			OMGT_DBGPRINT(port, "Completed request: OK\n");
		}
	}

	response = MemoryAllocate2AndClear(sizeof(STL_PA_IMAGE_ID_DATA), IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG);
	if (response == NULL) {
		OMGT_OUTPUT_ERROR(port, "error allocating response buffer\n");
		goto done;
	}
	memcpy((uint8 *)response, rsp_mad->Data, min(sizeof(STL_PA_IMAGE_ID_DATA), rcv_buf_len - IB_SA_DATA_OFFS));

	// translate the data.
	BSWAP_STL_PA_IMAGE_ID(response);
done:
	omgt_free_query_result_buffer(query_result);
	if (rsp_mad)
		free(rsp_mad);

	DBG_EXIT_FUNC(port);
	return response;
}

/**
 *  Move a frozen image 1 to image 2.
 *
 * @param port                  Local port to operate on. 
 * @param move_info             Ponter to move info (src and dest image ID). 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the image  move info.  The caller must free it.
 *     NULL         - Error
 */
STL_MOVE_FREEZE_DATA *
iba_pa_single_mad_move_freeze_response_query(
    IN struct omgt_port  *port,
    IN STL_MOVE_FREEZE_DATA *move_info
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    STL_MOVE_FREEZE_DATA        *response = NULL, *p;
    SA_MAD                      *rsp_mad = NULL;
    size_t                      rcv_buf_len = 0;
    uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_MOVE_FREEZE_DATA)] = {0};

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return response;

    p = (STL_MOVE_FREEZE_DATA *)(((SA_MAD *)request_data)->Data);
    p->oldFreezeImage.imageNumber = hton64(move_info->oldFreezeImage.imageNumber);
    p->oldFreezeImage.imageOffset = hton32(move_info->oldFreezeImage.imageOffset);
    p->newFreezeImage.imageNumber = hton64(move_info->newFreezeImage.imageNumber);
    p->newFreezeImage.imageOffset = hton32(move_info->newFreezeImage.imageOffset);

	// submit request
	fstatus = pa_query_common(port, STL_PA_CMD_SET, STL_PA_ATTRID_MOVE_FREEZE_FRAME, 0,
		request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
	if (fstatus != FSUCCESS) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
		goto done;
	} else if (rsp_mad->SaHdr.AttributeOffset) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, unexpected multiple MAD response\n");
		goto done;
	} else {
		if (port->pa_verbose) {
			OMGT_DBGPRINT(port, "Completed request: OK\n");
		}
	}

	response = MemoryAllocate2AndClear(sizeof(STL_MOVE_FREEZE_DATA), IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG);
	if (response == NULL) {
		OMGT_OUTPUT_ERROR(port, "error allocating response buffer\n");
		goto done;
	}
	memcpy((uint8 *)response, rsp_mad->Data, min(sizeof(STL_MOVE_FREEZE_DATA), rcv_buf_len - IB_SA_DATA_OFFS));

	// translate the data.
	BSWAP_STL_PA_MOVE_FREEZE(response);
done:
	omgt_free_query_result_buffer(query_result);
	if (rsp_mad)
		free(rsp_mad);

	DBG_EXIT_FUNC(port);
	return response;
}

/**
 *  Get image info
 *
 * @param port                  Local port to operate on. 
 * @param image_info            Ponter to image info (containing valid image ID). 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the image info data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_IMAGE_INFO_DATA *
iba_pa_multi_mad_get_image_info_response_query (
    IN struct omgt_port  *port,
    IN STL_PA_IMAGE_INFO_DATA  *image_info
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    STL_PA_IMAGE_INFO_DATA      *response = NULL, *p;
    SA_MAD                      *rsp_mad = NULL;
    size_t                      rcv_buf_len = 0;
    uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_IMAGE_INFO_DATA)] = {0};

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return response;

    p = (STL_PA_IMAGE_INFO_DATA *)(((SA_MAD *)request_data)->Data);
    p->imageId.imageNumber = hton64(image_info->imageId.imageNumber);
    p->imageId.imageOffset = hton32(image_info->imageId.imageOffset);
	p->imageId.imageTime.absoluteTime = hton32(image_info->imageId.imageTime.absoluteTime);
	p->reserved = 0;

	// submit request
	fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_IMAGE_INFO, 0,
		request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
	if (fstatus != FSUCCESS) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
		goto done;
	} else {
		if (port->pa_verbose) {
			OMGT_DBGPRINT(port, "Completed request: OK\n");
		}
	}

	response = MemoryAllocate2AndClear(sizeof(STL_PA_IMAGE_INFO_DATA), IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG);
	if (response == NULL) {
		OMGT_OUTPUT_ERROR(port, "error allocating response buffer\n");
		goto done;
	}
	memcpy((uint8 *)response, rsp_mad->Data, min(sizeof(STL_PA_IMAGE_INFO_DATA), rcv_buf_len - IB_SA_DATA_OFFS));

	// translate the data.
	BSWAP_STL_PA_IMAGE_INFO(response);
done:
	omgt_free_query_result_buffer(query_result);
	if (rsp_mad)
		free(rsp_mad);

	DBG_EXIT_FUNC(port);
	return response;
}

/**
 *  Get multi-record response for pa group data (group list).
 *
 * @param port                      Local port to operate on.
 * @param query                     Pointer to the query
 * @param pquery_result             Pointer to query result to be filled. The caller
 *                                  has to free the buffer.
 *
 * @return
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS
iba_pa_multi_mad_group_list_response_query(
	IN struct omgt_port       *port,
	IN POMGT_QUERY            query,
	OUT PQUERY_RESULT_VALUES  *pquery_result
	)
{
	FSTATUS                     fstatus = FSUCCESS;
	QUERY_RESULT_VALUES         *query_result = NULL;
	SA_MAD                      *rsp_mad = NULL;
	size_t                      rcv_buf_len = 0;
	uint8_t                     request_data[PA_REQ_HEADER_SIZE] = { 0 };
	STL_PA_GROUP_LIST           *pa_data;
	STL_PA_GROUP_LIST_RESULTS   *pa_result;
	int i;

	DBG_ENTER_FUNC(port);
	// Check the incoming port parameter
	if (port == NULL) return FERROR;

	// process the command.
	switch (query->OutputType) {
	case OutputTypePaTableRecord:
		// submit request
		fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_GRP_LIST, 0,
			request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
		if (fstatus != FSUCCESS) {
			if (port->pa_verbose)
				OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n", (unsigned int)fstatus);
			goto done;
		} else {
			if (port->pa_verbose) {
				OMGT_DBGPRINT(port, "Completed request: OK\n");
			}
		}

		// translate the data.
		pa_result = (STL_PA_GROUP_LIST_RESULTS *)query_result->QueryResult;
		pa_data = pa_result->GroupListRecords;
		for (i = 0; i < pa_result->NumGroupListRecords; i++) {
			STL_PA_GROUP_LIST *groupname = (STL_PA_GROUP_LIST *)GET_RESULT_OFFSET(rsp_mad, i);
			pa_data[i] = *groupname;
		}
		break;

	default:
		OMGT_OUTPUT_ERROR(port, "Query Not supported in OPAMGT: Input=%s, Output=%s\n",
			iba_pa_query_input_type_msg(query->InputType),
			iba_pa_query_result_type_msg(query->OutputType));
		fstatus = FERROR;
		goto done;
		break;
	}

done:
	if (rsp_mad)
		free(rsp_mad);

	*pquery_result = query_result;

	DBG_EXIT_FUNC(port);
	return fstatus;
}
FSTATUS
iba_pa_multi_mad_group_list2_response_query(
	IN struct omgt_port       *port,
	IN POMGT_QUERY            query,
	IN STL_PA_IMAGE_ID_DATA   *imageId,
	OUT PQUERY_RESULT_VALUES  *pquery_result
	)
{
	FSTATUS                     fstatus = FSUCCESS;
	QUERY_RESULT_VALUES         *query_result = NULL;
	SA_MAD                      *rsp_mad = NULL;
	size_t                      rcv_buf_len = 0;
	uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_IMAGE_ID_DATA)] = { 0 };
	STL_PA_GROUP_LIST2          *pa_data;
	STL_PA_GROUP_LIST2_RESULTS  *pa_result;
	STL_PA_IMAGE_ID_DATA        *p;
	int i;

	DBG_ENTER_FUNC(port);
	// Check the incoming port parameter
	if (port == NULL) return FERROR;

	p = (STL_PA_IMAGE_ID_DATA *)(((SA_MAD *)request_data)->Data);
	*p = *imageId;
	BSWAP_STL_PA_IMAGE_ID(p);

	// process the command.
	switch (query->OutputType) {
	case OutputTypePaTableRecord:
		// submit request
		fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_GRP_LIST2, 0,
			request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
		if (fstatus != FSUCCESS) {
			if (port->pa_verbose)
				OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n", (unsigned int)fstatus);
			goto done;
		} else {
			if (port->pa_verbose) {
				OMGT_DBGPRINT(port, "Completed request: OK\n");
			}
		}

		// translate the data.
		pa_result = (STL_PA_GROUP_LIST2_RESULTS *)query_result->QueryResult;
		pa_data = pa_result->GroupList2Records;
		for (i = 0; i < pa_result->NumGroupList2Records; i++) {
			STL_PA_GROUP_LIST2 *group = (STL_PA_GROUP_LIST2 *)GET_RESULT_OFFSET(rsp_mad, i);
			BSWAP_STL_PA_IMAGE_ID(&group->imageId);
			pa_data[i] = *group;
		}
		break;

	default:
		OMGT_OUTPUT_ERROR(port, "Query Not supported in OPAMGT: Input=%s, Output=%s\n",
			iba_pa_query_input_type_msg(query->InputType),
			iba_pa_query_result_type_msg(query->OutputType));
		fstatus = FERROR;
		goto done;
		break;
	}

done:
	if (rsp_mad)
		free(rsp_mad);

	*pquery_result = query_result;

	DBG_EXIT_FUNC(port);
	return fstatus;
}
/**
 *  Get multi-record response for pa group info (stats).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param group_name                Group name 
 * @param pquery_result             Pointer to query result to be filled. The caller has to 
 *                                  to free the buffer.
 * @param image_id                  Pointer to the image ID. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_group_stats_response_query(
    IN struct omgt_port              *port,
    IN POMGT_QUERY                   query,
    IN char                         *group_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN STL_PA_IMAGE_ID_DATA         *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    SA_MAD                      *rsp_mad = NULL;
    size_t                      rcv_buf_len = 0;
    uint8_t                     request_data[PA_REQ_HEADER_SIZE + 80] = {0}; // 80 = GroupName + ImageId (Request)
    STL_PA_PM_GROUP_INFO_DATA   *pa_data, *p;
    STL_PA_GROUP_INFO_RESULTS   *pa_result;
	int i;

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return FERROR;
    
    p = (STL_PA_PM_GROUP_INFO_DATA *)(((SA_MAD *)request_data)->Data);
    snprintf(p->groupName, sizeof(p->groupName), "%s", group_name);
    p->imageId.imageNumber = image_id->imageNumber;
    p->imageId.imageOffset = image_id->imageOffset;
	p->imageId.imageTime.absoluteTime = image_id->imageTime.absoluteTime;
	BSWAP_STL_PA_PM_GROUP_INFO(p, 1);

	// process the command.
	switch (query->OutputType) {
	case OutputTypePaTableRecord:
		// submit request
		fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_GRP_INFO, 0,
			request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
		if (fstatus != FSUCCESS){
			if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",(unsigned int)fstatus);
			goto done;
		} else {
			if (port->pa_verbose){
				OMGT_DBGPRINT(port, "Completed request: OK\n");
			}
		}

		// translate the data.
		pa_result = (STL_PA_GROUP_INFO_RESULTS *)query_result->QueryResult;
		pa_data = pa_result->GroupInfoRecords;
		for (i = 0; i < pa_result->NumGroupInfoRecords; i++) {
			STL_PA_PM_GROUP_INFO_DATA *group = (STL_PA_PM_GROUP_INFO_DATA *)GET_RESULT_OFFSET(rsp_mad, i);
			pa_data[i] = *group;
			BSWAP_STL_PA_PM_GROUP_INFO(&pa_data[i], 0);
		}
		break;

	default:
		OMGT_OUTPUT_ERROR(port, "Query Not supported in OPAMGT: Input=%s, Output=%s\n",
			iba_pa_query_input_type_msg(query->InputType),
			iba_pa_query_result_type_msg(query->OutputType));
		fstatus = FERROR;
		goto done;
		break;
	}

done:
	if (rsp_mad)
		free(rsp_mad);

	*pquery_result = query_result;

	DBG_EXIT_FUNC(port);
	return fstatus;
}

/**
 *  Get multi-record response for pa group config.
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param group_name                Group name 
 * @param pquery_result             Pointer to query result to be filled. The caller has 
 *                                  to free the buffer.
 * @param imageID                   Pointer to the image ID. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_group_config_response_query(
    IN struct omgt_port              *port,
    IN POMGT_QUERY             query,
    IN char                         *group_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN STL_PA_IMAGE_ID_DATA         *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    SA_MAD                      *rsp_mad = NULL;
    size_t                      rcv_buf_len = 0;
    uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_PM_GROUP_CFG_REQ)] = {0};
    STL_PA_PM_GROUP_CFG_RSP     *pa_data;
	STL_PA_PM_GROUP_CFG_REQ     *p;
    STL_PA_GROUP_CONFIG_RESULTS *pa_result;
	int	i;

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return FERROR;
    
    p = (STL_PA_PM_GROUP_CFG_REQ *)(((SA_MAD *)request_data)->Data);
    snprintf(p->groupName, sizeof(p->groupName), "%s", group_name);

    p->imageId.imageNumber = hton64(image_id->imageNumber);
    p->imageId.imageOffset = hton32(image_id->imageOffset);
	p->imageId.imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);
    
    // process the command.
	switch (query->OutputType) {
	case OutputTypePaTableRecord:
		// submit request
		fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_GRP_CFG, 0,
			request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
		if (fstatus != FSUCCESS){
			if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",(unsigned int)fstatus);
			goto done;
		} else {
			if (port->pa_verbose){
				OMGT_DBGPRINT(port, "Completed request: OK\n");
			}
		}

		// translate the data.
		pa_result = (STL_PA_GROUP_CONFIG_RESULTS *)query_result->QueryResult;
		pa_data = pa_result->GroupConfigRecords;
		for (i = 0; i < pa_result->NumGroupConfigRecords; i++) {
			STL_PA_PM_GROUP_CFG_RSP *group_cfg = (STL_PA_PM_GROUP_CFG_RSP *)GET_RESULT_OFFSET(rsp_mad, i);
			pa_data[i] = *group_cfg;
			BSWAP_STL_PA_GROUP_CONFIG_RSP(&pa_data[i]);
		}
		break;

	default:
		OMGT_OUTPUT_ERROR(port, "Query Not supported in OPAMGT: Input=%s, Output=%s\n",
			iba_pa_query_input_type_msg(query->InputType),
			iba_pa_query_result_type_msg(query->OutputType));
		fstatus = FERROR;
		goto done;
		break;
	}

done:
	if (rsp_mad)
		free(rsp_mad);

	*pquery_result = query_result;

	DBG_EXIT_FUNC(port);
	return fstatus;
}	// End of iba_pa_multi_mad_group_config_response_query()

/**
 *  Get multi-record response for pa group node Info.
 *
 * @param port           	Local port to operate on.
 * @param query                 Pointer to the query
 * @param group_name 		Group name
 * @param nodeLid               node LID
 * @param nodeGuid              node GUID
 * @param nodeDesc              node Description
 * @param pquery_result       	Pointer to query result to be filled. The caller has
 *                              to free the buffer.
 * @param imageID               Pointer to the image ID.
 *
 * @return
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS
iba_pa_multi_mad_group_nodeinfo_response_query(
	IN struct omgt_port              *port,
	IN POMGT_QUERY                   query,
	IN char                         *group_name,
	IN STL_LID                      nodeLid,
	IN uint64                       nodeGuid,
	IN char                         *nodeDesc,
	OUT PQUERY_RESULT_VALUES        *pquery_result,
	IN STL_PA_IMAGE_ID_DATA         *image_id
	)
{
	FSTATUS                     fstatus = FSUCCESS;
	QUERY_RESULT_VALUES         *query_result = NULL;
	SA_MAD                      *rsp_mad = NULL;
	size_t                      rcv_buf_len = 0;
	uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_GROUP_NODEINFO_REQ)] = {0};
	STL_PA_GROUP_NODEINFO_RSP     *pa_data;
	STL_PA_GROUP_NODEINFO_REQ     *p;
	STL_PA_GROUP_NODEINFO_RESULTS *pa_result;
	int i;

	DBG_ENTER_FUNC(port);
	// Check the incoming port parameter
	if (port == NULL) return FERROR;

	p = (STL_PA_GROUP_NODEINFO_REQ *)(((SA_MAD *)request_data)->Data);
	snprintf(p->groupName, sizeof(p->groupName), "%s", group_name);
	p->nodeLID = nodeLid;
	p->nodeGUID = nodeGuid;
	if(nodeDesc)
		snprintf(p->nodeDesc, sizeof(p->nodeDesc), "%s", nodeDesc);
	p->imageId.imageNumber = image_id->imageNumber;
	p->imageId.imageOffset = image_id->imageOffset;
	p->imageId.imageTime.absoluteTime = image_id->imageTime.absoluteTime;
	BSWAP_STL_PA_GROUP_NODE_INFO_REQ(p);

	// process the command.
	switch (query->OutputType) {
	case OutputTypePaTableRecord:
		// submit request
		fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_GRP_NODE_INFO, 0,
			request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
		if (fstatus != FSUCCESS){
			if (port->pa_verbose)
				OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",(unsigned int)fstatus);
			goto done;
		} else {
			if (port->pa_verbose){
				OMGT_DBGPRINT(port, "Completed request: OK\n");
			}
		}

		// translate the data.
		pa_result = (STL_PA_GROUP_NODEINFO_RESULTS *)query_result->QueryResult;
		pa_data = pa_result->GroupNodeInfoRecords;
		for (i = 0; i < pa_result->NumGroupNodeInfoRecords; i++) {
			STL_PA_GROUP_NODEINFO_RSP *group_node_info = (STL_PA_GROUP_NODEINFO_RSP *)GET_RESULT_OFFSET(rsp_mad, i);
			pa_data[i] = *group_node_info;
			BSWAP_STL_PA_GROUP_NODE_INFO_RSP(&pa_data[i]);
		}
		break;

	default:
		OMGT_OUTPUT_ERROR(port, "Query Not supported in OPAMGT: Input=%s, Output=%s\n",
		iba_pa_query_input_type_msg(query->InputType),
		iba_pa_query_result_type_msg(query->OutputType));
		fstatus = FERROR;
		goto done;
		break;
	}

done:
	if (rsp_mad)
		free(rsp_mad);

	*pquery_result = query_result;

	DBG_EXIT_FUNC(port);
	return fstatus;
} // End of iba_pa_multi_mad_group_nodeinfo_response_query()

/**
 *  Get multi-record response for pa group link Info.
 *
 * @param port        		Local port to operate on.
 * @param query                 Pointer to the query
 * @param group_name         	Group name
 * @param inputLid              input LID
 * @param inputPort          	input port
 * @param pquery_result       	Pointer to query result to be filled. The caller has
 *                              to free the buffer.
 * @param imageID               Pointer to the image ID.
 *
 * @return
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS
iba_pa_multi_mad_group_linkinfo_response_query(
	IN struct omgt_port       	*port,
	IN POMGT_QUERY             	query,
	IN char                         *group_name,
	IN STL_LID                      inputLid,
	IN uint8                      	inputPort,
	OUT PQUERY_RESULT_VALUES        *pquery_result,
	IN STL_PA_IMAGE_ID_DATA         *image_id
	)
{
	FSTATUS                     fstatus = FSUCCESS;
	QUERY_RESULT_VALUES         *query_result = NULL;
	SA_MAD                      *rsp_mad = NULL;
	size_t                      rcv_buf_len = 0;
	uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_GROUP_LINKINFO_REQ)] = {0};
	STL_PA_GROUP_LINKINFO_RSP     *pa_data;
	STL_PA_GROUP_LINKINFO_REQ     *p;
	STL_PA_GROUP_LINKINFO_RESULTS *pa_result;
	int i;

	DBG_ENTER_FUNC(port);
	// Check the incoming port parameter
	if (port == NULL) return FERROR;

	p = (STL_PA_GROUP_LINKINFO_REQ *)(((SA_MAD *)request_data)->Data);
	snprintf(p->groupName, sizeof(p->groupName), "%s", group_name);
	p->lid = inputLid;
	p->port = inputPort;
	p->imageId.imageNumber = image_id->imageNumber;
	p->imageId.imageOffset = image_id->imageOffset;
	p->imageId.imageTime.absoluteTime = image_id->imageTime.absoluteTime;
	BSWAP_STL_PA_GROUP_LINK_INFO_REQ(p);

	// process the command.
	switch (query->OutputType) {
	case OutputTypePaTableRecord:
		// submit request
		fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_GRP_LINK_INFO, 0,
			request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
		if (fstatus != FSUCCESS){
			if (port->pa_verbose)
				OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",(unsigned int)fstatus);
			goto done;
		} else {
			if (port->pa_verbose){
				OMGT_DBGPRINT(port, "Completed request: OK\n");
			}
		}

		// translate the data.
		pa_result = (STL_PA_GROUP_LINKINFO_RESULTS *)query_result->QueryResult;
		pa_data = pa_result->GroupLinkInfoRecords;
		for (i = 0; i < pa_result->NumGroupLinkInfoRecords; i++) {
			STL_PA_GROUP_LINKINFO_RSP *group_link_info = (STL_PA_GROUP_LINKINFO_RSP *)GET_RESULT_OFFSET(rsp_mad, i);
			pa_data[i] = *group_link_info;
			BSWAP_STL_PA_GROUP_LINK_INFO_RSP(&pa_data[i]);
		}
		break;

	default:
		OMGT_OUTPUT_ERROR(port, "Query Not supported in OPAMGT: Input=%s, Output=%s\n",
		iba_pa_query_input_type_msg(query->InputType),
		iba_pa_query_result_type_msg(query->OutputType));
		fstatus = FERROR;
		goto done;
		break;
	}

done:
	if (rsp_mad)
		free(rsp_mad);

	*pquery_result = query_result;

	DBG_EXIT_FUNC(port);
	return fstatus;
}


/**
 *  Get multi-record response for pa group focus portlist.
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param group_name                Group name 
 * @param select                    Select value for focus portlist. 
 * @param start                     Start index value of portlist 
 * @param range                     Index range of portlist. 
 * @param pquery_result             Pointer to query result to be filled. The caller has 
 *                                  to fill the buffer.
 * @param imageID                   Pointer to the image ID. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_focus_ports_response_query (
    IN struct omgt_port              *port,
    IN POMGT_QUERY             query,
    IN char                         *group_name,
    IN uint32                       select,
    IN uint32                       start,
    IN uint32                       range,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN STL_PA_IMAGE_ID_DATA         *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    SA_MAD                      *rsp_mad = NULL;
    size_t                      rcv_buf_len = 0;
    uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_FOCUS_PORTS_REQ)] = {0};
    STL_FOCUS_PORTS_RSP         *pa_data;
	STL_FOCUS_PORTS_REQ         *p;
    STL_PA_FOCUS_PORTS_RESULTS  *pa_result;
    int i;

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return FERROR;
    
    p = (STL_FOCUS_PORTS_REQ *)(((SA_MAD *)request_data)->Data);
    snprintf(p->groupName, sizeof(p->groupName), "%s", group_name);
    p->select = hton32(select);
    p->start = hton32(start);
    p->range = hton32(range);

    p->imageId.imageNumber = hton64(image_id->imageNumber);
    p->imageId.imageOffset = hton32(image_id->imageOffset);
    p->imageId.imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);

	// process the command.
	switch (query->OutputType) {
	case OutputTypePaTableRecord:
		// submit request
		fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_FOCUS_PORTS, 0,
			request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
		if (fstatus != FSUCCESS) {
			if (port->pa_verbose)
				OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n", (unsigned int)fstatus);
			goto done;
		} else {
			if (port->pa_verbose) {
				OMGT_DBGPRINT(port, "Completed request: OK\n");
			}
		}

		// translate the data.
		pa_result = (STL_PA_FOCUS_PORTS_RESULTS *)query_result->QueryResult;
		pa_data = pa_result->FocusPortsRecords;
		for (i = 0; i < pa_result->NumFocusPortsRecords; i++) {
			STL_FOCUS_PORTS_RSP *focus_port = (STL_FOCUS_PORTS_RSP *)GET_RESULT_OFFSET(rsp_mad, i);
			pa_data[i] = *focus_port;
			BSWAP_STL_PA_FOCUS_PORTS_RSP(&pa_data[i]);
		}
		break;

	default:
		OMGT_OUTPUT_ERROR(port, "Query Not supported in OPAMGT: Input=%s, Output=%s\n",
			iba_pa_query_input_type_msg(query->InputType),
			iba_pa_query_result_type_msg(query->OutputType));
		fstatus = FERROR;
		goto done;
		break;
	}

done:
	if (rsp_mad)
		free(rsp_mad);

	*pquery_result = query_result;

	DBG_EXIT_FUNC(port);
	return fstatus;
}


/**
 *  Get multi-record response for pa group focus portlist.
 *
 * @param port              Local port to operate on.
 * @param query             Pointer to the query.
 * @param group_name        Group name.
 * @param select            Select value for focus portlist.
 * @param start             Start index value of portlist.
 * @param range             Index range of portlist.
 * @param pquery_result     Pointer to query result to be filled. The caller has
 *  						to fill the buffer.
 * @param imageID           Pointer to the image ID.
 *
 * @return
 *   FSUCCESS	- Get successfully
 *     FERROR	- Error
 */
FSTATUS
iba_pa_multi_mad_focus_ports_multiselect_response_query (
	IN struct omgt_port		    *port,
	IN POMGT_QUERY              query,
	IN char				        *group_name,
	IN uint32			        start,
	IN uint32			        range,
	OUT PQUERY_RESULT_VALUES	*pquery_result,
	IN STL_PA_IMAGE_ID_DATA		*image_id,
	IN STL_FOCUS_PORT_TUPLE		*tuple,
	IN uint8			        logical_operator
	)
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    SA_MAD                      *rsp_mad = NULL;
    size_t                      rcv_buf_len = 0;
    uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_FOCUS_PORTS_MULTISELECT_REQ)] = {0};
    STL_FOCUS_PORTS_MULTISELECT_RSP         *pa_data;
	STL_FOCUS_PORTS_MULTISELECT_REQ         *p;
    STL_PA_FOCUS_PORTS_MULTISELECT_RESULTS  *pa_result;
    int i;

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return FERROR;
    
    p = (STL_FOCUS_PORTS_MULTISELECT_REQ *)(((SA_MAD *)request_data)->Data);
	snprintf(p->groupName, STL_PM_GROUPNAMELEN, "%s", group_name);
	p->start = hton32(start);
	p->range = hton32(range);

	for (i = 0; i < MAX_NUM_FOCUS_PORT_TUPLES; i++) {
		p->tuple[i].select = hton32(tuple[i].select);
		p->tuple[i].comparator = tuple[i].comparator;
		p->tuple[i].argument = hton64(tuple[i].argument);
	}

	p->logical_operator = logical_operator;

	p->imageId.imageNumber = hton64(image_id->imageNumber);
	p->imageId.imageOffset = hton32(image_id->imageOffset);
	p->imageId.imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);

	// process the command.
	switch (query->OutputType) {
	case OutputTypePaTableRecord:
		// submit request
		fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_FOCUS_PORTS_MULTISELECT, 0,
			request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
		if (fstatus != FSUCCESS) {
			if (port->pa_verbose)
				OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n", (unsigned int)fstatus);
			goto done;
		} else {
			if (port->pa_verbose) {
				OMGT_DBGPRINT(port, "Completed request: OK\n");
			}
		}

		// translate the data.
		pa_result = (STL_PA_FOCUS_PORTS_MULTISELECT_RESULTS *)query_result->QueryResult;
		pa_data = pa_result->FocusPortsRecords;
		for (i = 0; i < pa_result->NumFocusPortsRecords; i++) {
			STL_FOCUS_PORTS_MULTISELECT_RSP *focus_port = (STL_FOCUS_PORTS_MULTISELECT_RSP *)GET_RESULT_OFFSET(rsp_mad, i);
			pa_data[i] = *focus_port;
			BSWAP_STL_PA_FOCUS_PORTS_MULTISELECT_RSP(&pa_data[i]);
		}
		break;

	default:
		OMGT_OUTPUT_ERROR(port, "Query Not supported in OPAMGT: Input=%s, Output=%s\n",
			iba_pa_query_input_type_msg(query->InputType),
			iba_pa_query_result_type_msg(query->OutputType));
		fstatus = FERROR;
		goto done;
		break;
	}

done:
	if (rsp_mad)
		free(rsp_mad);

	*pquery_result = query_result;

	DBG_EXIT_FUNC(port);
	return fstatus;
}

/**
 *  Get multi-record response for pa vf data (vf list).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param pquery_result             Pointer to query result to be filled. The caller 
 *                                  has to free the buffer.
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_vf_list_response_query(
    IN struct omgt_port              *port,
    IN POMGT_QUERY             query,
    OUT PQUERY_RESULT_VALUES        *pquery_result
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    SA_MAD                      *rsp_mad = NULL;
    size_t                      rcv_buf_len = 0;
	uint8_t                     request_data[PA_REQ_HEADER_SIZE] = {0};
    STL_PA_VF_LIST	        	*pa_data;
    STL_PA_VF_LIST_RESULTS		*pa_result;
	int i;

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return FERROR;
    
	// process the command.
	switch (query->OutputType){
	case OutputTypePaTableRecord:
		// submit request
		fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_VF_LIST, 0,
			request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
		if (fstatus != FSUCCESS){
			if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",(unsigned int)fstatus);
			goto done;
		} else {
			if (port->pa_verbose){
				OMGT_DBGPRINT(port, "Completed request: OK\n");
			}
		}

		// translate the data.
		pa_result = (STL_PA_VF_LIST_RESULTS *)query_result->QueryResult;
		pa_data = pa_result->VFListRecords;
		for (i = 0; i < pa_result->NumVFListRecords; i++) {
			STL_PA_VF_LIST *vfname = (STL_PA_VF_LIST *)GET_RESULT_OFFSET(rsp_mad, i);
			pa_data[i] = *vfname;
		}
		break;

	default:
		OMGT_OUTPUT_ERROR(port, "Query Not supported in OPAMGT: Input=%s, Output=%s\n",
			iba_pa_query_input_type_msg(query->InputType),
			iba_pa_query_result_type_msg(query->OutputType));
		fstatus = FERROR;
		goto done;
		break;
	}

done:
	if (rsp_mad)
		free(rsp_mad);

	*pquery_result = query_result;

	DBG_EXIT_FUNC(port);
	return fstatus;
}
FSTATUS
iba_pa_multi_mad_vf_list2_response_query(
	IN struct omgt_port       *port,
	IN POMGT_QUERY            query,
	IN STL_PA_IMAGE_ID_DATA   *imageId,
	OUT PQUERY_RESULT_VALUES  *pquery_result
	)
{
	FSTATUS                     fstatus = FSUCCESS;
	QUERY_RESULT_VALUES         *query_result = NULL;
	SA_MAD                      *rsp_mad = NULL;
	size_t                      rcv_buf_len = 0;
	uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_IMAGE_ID_DATA)] = { 0 };
	STL_PA_VF_LIST2             *pa_data;
	STL_PA_VF_LIST2_RESULTS     *pa_result;
	STL_PA_IMAGE_ID_DATA        *p;

	DBG_ENTER_FUNC(port);
	// Check the incoming port parameter
	if (port == NULL) return FERROR;

	p = (STL_PA_IMAGE_ID_DATA *)(((SA_MAD *)request_data)->Data);
	*p = *imageId;
	BSWAP_STL_PA_IMAGE_ID(p);

	// process the command.
	switch (query->OutputType) {
	case OutputTypePaTableRecord:
		// submit request
		fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_VF_LIST2, 0,
			request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
		if (fstatus != FSUCCESS) {
			if (port->pa_verbose)
				OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n", (unsigned int)fstatus);
			goto done;
		} else {
			if (port->pa_verbose) {
				OMGT_DBGPRINT(port, "Completed request: OK\n");
			}
		}

		int i;
		// translate the data.
		pa_result = (STL_PA_VF_LIST2_RESULTS *)query_result->QueryResult;
		pa_data = pa_result->VFList2Records;
		for (i = 0; i < pa_result->NumVFList2Records; i++) {
			STL_PA_VF_LIST2 *vf = (STL_PA_VF_LIST2 *)GET_RESULT_OFFSET(rsp_mad, i);
			BSWAP_STL_PA_IMAGE_ID(&vf->imageId);
			pa_data[i] = *vf;
		}
		break;

	default:
		OMGT_OUTPUT_ERROR(port, "Query Not supported in OPAMGT: Input=%s, Output=%s\n",
			iba_pa_query_input_type_msg(query->InputType),
			iba_pa_query_result_type_msg(query->OutputType));
		fstatus = FERROR;
		goto done;
		break;
	}

done:
	if (rsp_mad)
		free(rsp_mad);

	*pquery_result = query_result;

	DBG_EXIT_FUNC(port);
	return fstatus;
}
/**
 *  Get multi-record response for pa vf info (stats).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param vf_name                	VF name 
 * @param pquery_result             Pointer to query result to be filled. The caller has to 
 *                                  to free the buffer.
 * @param image_id                  Pointer to the image ID. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_vf_info_response_query(
    IN struct omgt_port              *port,
    IN POMGT_QUERY             query,
    IN char                         *vf_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN STL_PA_IMAGE_ID_DATA         *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    SA_MAD                      *rsp_mad = NULL;
    size_t                      rcv_buf_len = 0;
    uint8_t                     request_data[PA_REQ_HEADER_SIZE + 88] = {0}; //80 = VFName + reserved + ImageID
    STL_PA_VF_INFO_DATA         *pa_data, *p;
    STL_PA_VF_INFO_RESULTS      *pa_result;
	int i;

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return FERROR;
    
    p = (STL_PA_VF_INFO_DATA *)(((SA_MAD *)request_data)->Data);
    snprintf(p->vfName, sizeof(p->vfName), "%s", vf_name);
    p->imageId.imageNumber = image_id->imageNumber;
    p->imageId.imageOffset = image_id->imageOffset;
	p->imageId.imageTime.absoluteTime = image_id->imageTime.absoluteTime;
	BSWAP_STL_PA_VF_INFO(p, 1);
    
	// process the command.
	switch (query->OutputType) {
	case OutputTypePaTableRecord:
		// submit request
		fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_VF_INFO, 0,
			request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
		if (fstatus != FSUCCESS){
			if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",(unsigned int)fstatus);
			goto done;
		} else {
			if (port->pa_verbose){
				OMGT_DBGPRINT(port, "Completed request: OK\n");
			}
		}

		// translate the data.
		pa_result = (STL_PA_VF_INFO_RESULTS *)query_result->QueryResult;
		pa_data = pa_result->VFInfoRecords;
		for (i = 0; i < pa_result->NumVFInfoRecords; i++) {
			STL_PA_VF_INFO_DATA *vfinfo = (STL_PA_VF_INFO_DATA *)GET_RESULT_OFFSET(rsp_mad, i);
			pa_data[i] = *vfinfo;
			BSWAP_STL_PA_VF_INFO(&pa_data[i], 0);
		}
		break;

	default:
		OMGT_OUTPUT_ERROR(port, "Query Not supported in OPAMGT: Input=%s, Output=%s\n",
			iba_pa_query_input_type_msg(query->InputType),
			iba_pa_query_result_type_msg(query->OutputType));
		fstatus = FERROR;
		goto done;
		break;
	}

done:
	if (rsp_mad)
		free(rsp_mad);

	*pquery_result = query_result;

	DBG_EXIT_FUNC(port);
	return fstatus;
}

/**
 *  Get multi-record response for pa vf config.
 *
 * @param port                      Local port to operate on.
 * @param query                     Pointer to the query
 * @param vf_name                   VF Name
 * @param pquery_result             Pointer to query result to be filled. The
 *  								caller has to free the buffer.
 * @param image_id					Pointer to the image ID.
 *
 * @return
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_vf_config_response_query(
    IN struct omgt_port              *port,
    IN POMGT_QUERY             query,
    IN char                         *vf_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN STL_PA_IMAGE_ID_DATA         *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    SA_MAD                      *rsp_mad = NULL;
    size_t                      rcv_buf_len = 0;
    uint8_t                     request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_VF_CFG_REQ)] = {0};
    STL_PA_VF_CFG_RSP           *pa_data;
	STL_PA_VF_CFG_REQ           *p;
    STL_PA_VF_CONFIG_RESULTS    *pa_result;
	int	i;

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return FERROR;
    
    p = (STL_PA_VF_CFG_REQ *)(((SA_MAD *)request_data)->Data);
    snprintf(p->vfName, sizeof(p->vfName), "%s", vf_name);

    p->imageId.imageNumber = hton64(image_id->imageNumber);
    p->imageId.imageOffset = hton32(image_id->imageOffset);
    p->imageId.imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);

	// process the command.
	switch (query->OutputType) {
	case OutputTypePaTableRecord:
		// submit request
		fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_VF_CONFIG, 0,
			request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
		if (fstatus != FSUCCESS) {
			if (port->pa_verbose)
				OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n", (unsigned int)fstatus);
			goto done;
		} else {
			if (port->pa_verbose) {
				OMGT_DBGPRINT(port, "Completed request: OK\n");
			}
		}

		// translate the data.
		pa_result = (STL_PA_VF_CONFIG_RESULTS *)query_result->QueryResult;
		pa_data = pa_result->VFConfigRecords;
		for (i = 0; i < pa_result->NumVFConfigRecords; i++) {
			STL_PA_VF_CFG_RSP *vf_cfg = (STL_PA_VF_CFG_RSP *)GET_RESULT_OFFSET(rsp_mad, i);
			pa_data[i] = *vf_cfg;
			BSWAP_STL_PA_VF_CFG_RSP(&pa_data[i]);
		}
		break;

	default:
		OMGT_OUTPUT_ERROR(port, "Query Not supported in OPAMGT: Input=%s, Output=%s\n",
			iba_pa_query_input_type_msg(query->InputType),
			iba_pa_query_result_type_msg(query->OutputType));
		fstatus = FERROR;
		goto done;
		break;
	}

done:
	if (rsp_mad)
		free(rsp_mad);

	*pquery_result = query_result;

	DBG_EXIT_FUNC(port);
	return fstatus;
}

STL_PA_VF_PORT_COUNTERS_DATA *
iba_pa_single_mad_vf_port_counters_response_query(
    IN struct omgt_port      *port,
    IN STL_LID               node_lid,
    IN uint8_t               port_number,
    IN uint32_t              delta_flag,
    IN uint32_t              user_cntrs_flag,
    IN char                 *vfName,
    IN STL_PA_IMAGE_ID_DATA *image_id
    )
{
    FSTATUS                 fstatus = FSUCCESS;
    QUERY_RESULT_VALUES     *query_result = NULL;
    STL_PA_VF_PORT_COUNTERS_DATA *response = NULL, *p;
    SA_MAD                  *rsp_mad = NULL;
    size_t                  rcv_buf_len = 0;
    uint8_t                 request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_VF_PORT_COUNTERS_DATA)] = {0};

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return response;

    p = (STL_PA_VF_PORT_COUNTERS_DATA *)(((SA_MAD *)request_data)->Data);
    p->nodeLid = node_lid;
    p->portNumber = port_number;
    p->flags =  (delta_flag ? STL_PA_PC_FLAG_DELTA : 0) |
                (user_cntrs_flag ? STL_PA_PC_FLAG_USER_COUNTERS : 0);
    snprintf(p->vfName, STL_PM_VFNAMELEN, "%s", vfName);

    p->imageId.imageNumber = image_id->imageNumber;
    p->imageId.imageOffset = image_id->imageOffset;
	p->imageId.imageTime.absoluteTime = image_id->imageTime.absoluteTime;
	memset(p->reserved, 0, sizeof(p->reserved));
	p->reserved1 = 0;
    BSWAP_STL_PA_VF_PORT_COUNTERS(p);

	// submit request
	fstatus = pa_query_common(port, STL_PA_CMD_GET, STL_PA_ATTRID_GET_VF_PORT_CTRS, 0,
		request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
	if (fstatus != FSUCCESS) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
		goto done;
	} else if (rsp_mad->SaHdr.AttributeOffset) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, unexpected multiple MAD response\n");
		goto done;
	} else {
		if (port->pa_verbose) {
			OMGT_DBGPRINT(port, "Completed request: OK\n");
		}
	}

	response = MemoryAllocate2AndClear(sizeof(STL_PA_VF_PORT_COUNTERS_DATA), IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG);
	if (response == NULL) {
		OMGT_OUTPUT_ERROR(port, "error allocating response buffer\n");
		goto done;
	}
	memcpy((uint8 *)response, rsp_mad->Data, min(sizeof(STL_PA_VF_PORT_COUNTERS_DATA), rcv_buf_len - IB_SA_DATA_OFFS));

	// translate the data.
	BSWAP_STL_PA_VF_PORT_COUNTERS(response);
done:
	omgt_free_query_result_buffer(query_result);
	if (rsp_mad)
		free(rsp_mad);

	DBG_EXIT_FUNC(port);
	return response;
}

STL_PA_CLEAR_VF_PORT_COUNTERS_DATA *
iba_pa_single_mad_clr_vf_port_counters_response_query(
    IN struct omgt_port  *port,
    IN uint32_t         node_lid,
    IN uint8_t         port_number,
    IN uint32_t         select,
	IN char				*vfName
    )
{
    FSTATUS                 fstatus = FSUCCESS;
    QUERY_RESULT_VALUES     *query_result = NULL;
    STL_PA_CLEAR_VF_PORT_COUNTERS_DATA  *response = NULL, *p;
    SA_MAD                  *rsp_mad = NULL;
    size_t                  rcv_buf_len = 0;
    uint8_t                 request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_CLEAR_VF_PORT_COUNTERS_DATA)] = {0};

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return response;

    p = (STL_PA_CLEAR_VF_PORT_COUNTERS_DATA *)(((SA_MAD *)request_data)->Data);
    p->nodeLid = hton32(node_lid);
    p->portNumber = port_number;
    p->vfCounterSelectMask.AsReg32 = hton32(select);
    snprintf(p->vfName, STL_PM_VFNAMELEN, "%s", vfName);
	memset(p->reserved, 0, sizeof(p->reserved));
	p->reserved2 = 0;

	// submit request
	fstatus = pa_query_common(port, STL_PA_CMD_SET, STL_PA_ATTRID_CLR_VF_PORT_CTRS, 0,
		request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
	if (fstatus != FSUCCESS) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
		goto done;
	} else if (rsp_mad->SaHdr.AttributeOffset) {
		if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, unexpected multiple MAD response\n");
		goto done;
	} else {
		if (port->pa_verbose) {
			OMGT_DBGPRINT(port, "Completed request: OK\n");
		}
	}

	response = MemoryAllocate2AndClear(sizeof(STL_PA_CLEAR_VF_PORT_COUNTERS_DATA), IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG);
	if (response == NULL) {
		OMGT_OUTPUT_ERROR(port, "error allocating response buffer\n");
		goto done;
	}
	memcpy((uint8 *)response, rsp_mad->Data, min(sizeof(STL_PA_CLEAR_VF_PORT_COUNTERS_DATA), rcv_buf_len - IB_SA_DATA_OFFS));

	// translate the data.
	BSWAP_STL_PA_CLEAR_VF_PORT_COUNTERS(response);
done:
	omgt_free_query_result_buffer(query_result);
	if (rsp_mad)
		free(rsp_mad);

	DBG_EXIT_FUNC(port);
	return response;
}

FSTATUS 
iba_pa_multi_mad_vf_focus_ports_response_query (
    IN struct omgt_port              *port,
    IN POMGT_QUERY             query,
    IN char                         *vf_name,
    IN uint32                       select,
    IN uint32                       start,
    IN uint32                       range,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN STL_PA_IMAGE_ID_DATA         *image_id
    )
{
    FSTATUS                     	fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         	*query_result = NULL;
    SA_MAD                          *rsp_mad = NULL;
    size_t                         	rcv_buf_len = 0;
    uint8                        	request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_VF_FOCUS_PORTS_REQ)] = {0};
    STL_PA_VF_FOCUS_PORTS_RSP     	*pa_data;
	STL_PA_VF_FOCUS_PORTS_REQ       *p;
    STL_PA_VF_FOCUS_PORTS_RESULTS 	*pa_result;
    int i;

    DBG_ENTER_FUNC(port);
    // Check the incoming port parameter
    if (port == NULL) return FERROR;
    
    p = (STL_PA_VF_FOCUS_PORTS_REQ *)(((SA_MAD *)request_data)->Data);
    snprintf(p->vfName, sizeof(p->vfName), "%s", vf_name);
    p->select = hton32(select);
    p->start = hton32(start);
    p->range = hton32(range);

    p->imageId.imageNumber = hton64(image_id->imageNumber);
    p->imageId.imageOffset = hton32(image_id->imageOffset);
	p->imageId.imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);

	// process the command.
	switch (query->OutputType) {
	case OutputTypePaTableRecord:
		// submit request
		fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_VF_FOCUS_PORTS, 0,
			request_data, sizeof(request_data), &rcv_buf_len, &rsp_mad, &query_result);
		if (fstatus != FSUCCESS){
			if (port->pa_verbose)
			OMGT_OUTPUT_ERROR(port, "Error, requeset failed: status=%u\n",(unsigned int)fstatus);
			goto done;
		} else {
			if (port->pa_verbose){
				OMGT_DBGPRINT(port, "Completed request: OK\n");
			}
		}

		// translate the data.
		pa_result = (STL_PA_VF_FOCUS_PORTS_RESULTS *)query_result->QueryResult;
		pa_data = pa_result->FocusPortsRecords;
		for (i = 0; i < pa_result->NumVFFocusPortsRecords; i++) {
			STL_PA_VF_FOCUS_PORTS_RSP *vf_focus = (STL_PA_VF_FOCUS_PORTS_RSP *)GET_RESULT_OFFSET(rsp_mad, i);
			pa_data[i] = *vf_focus;
			BSWAP_STL_PA_VF_FOCUS_PORTS_RSP(&pa_data[i]);
		}
		break;

	default:
		OMGT_OUTPUT_ERROR(port, "Query Not supported in OPAMGT: Input=%s, Output=%s\n",
			iba_pa_query_input_type_msg(query->InputType),
			iba_pa_query_result_type_msg(query->OutputType));
		fstatus = FERROR;
		goto done;
		break;
	}

done:
	if (rsp_mad)
		free(rsp_mad);

	*pquery_result = query_result;

	DBG_EXIT_FUNC(port);
	return fstatus;
}

/**
 * @brief Get LID and SL of master PM service
 *
 * This function will query the SA service records to get the PM's GID. It will
 * then construct a PATH record query (using the ServiceGID as the DGID for the
 * request) to get the PM's LID and SL for sending PA queries.
 *
 * @param port         port object to to store master PM LID and SL.
 *
 * @return FSTATUS
 */
FSTATUS 
iba_pa_query_master_pm_lid(struct omgt_port *port)
{
    FSTATUS                 status;
    OMGT_QUERY         query;
    PQUERY_RESULT_VALUES    query_results = NULL;
    PQUERY_RESULT_VALUES    query_path_results = NULL;
    SERVICE_RECORD_RESULTS  *service_record_results;

    // Check the incoming port parameter
    if (NULL == port) {
        return (FERROR);
    }

    // query SA for Service Records to all PMs within the fabric
    memset(&query, 0, sizeof(query));
    query.InputType = InputTypeNoInput;
    query.OutputType = OutputTypeServiceRecord;

	status = omgt_query_sa(port, &query, &query_results);
	if (status != FSUCCESS) {
		OMGT_OUTPUT_ERROR(port, "Error, failed to get service records (status=0x%x) query_results=%p: %s\n",
			(unsigned int)status, (void *)query_results, FSTATUS_MSG(status));
	} else if ((query_results == NULL) || (query_results->ResultDataSize == 0)) {
		status = FUNAVAILABLE;
	} else {
		int i, ix, pmCount = 0;

		status = FUNAVAILABLE;
		service_record_results = (SERVICE_RECORD_RESULTS *)query_results->QueryResult;
		if (service_record_results->NumServiceRecords) {
			OMGT_DBGPRINT(port, "Got Service Records: records=%d\n", service_record_results->NumServiceRecords);
			for (i = 0; i < service_record_results->NumServiceRecords; ++i) {
				if (STL_PM_SERVICE_ID == service_record_results->ServiceRecords[i].RID.ServiceID) {
					pmCount++;
					if ((service_record_results->ServiceRecords[i].ServiceData8[0] >= STL_PM_VERSION) &&
						(service_record_results->ServiceRecords[i].ServiceData8[1] == STL_PM_MASTER)) {
						OMGT_DBGPRINT(port, "This is the Primary PM.\n");

						//
						// query SA for Path Record to the primary PM
						memset(&query, 0, sizeof(query));
						{
							query.InputType = InputTypePathRecord;
							query.OutputType = OutputTypePathRecord;
							query.InputValue.IbPathRecord.PathRecord.PathRecord.DGID =
								service_record_results->ServiceRecords[i].RID.ServiceGID;
							omgt_port_get_port_prefix(port,
								&query.InputValue.IbPathRecord.PathRecord.PathRecord.SGID.Type.Global.SubnetPrefix);
							omgt_port_get_port_guid(port,
								&query.InputValue.IbPathRecord.PathRecord.PathRecord.SGID.Type.Global.InterfaceID);
							query.InputValue.IbPathRecord.PathRecord.PathRecord.ServiceID = STL_PM_SERVICE_ID;

							query.InputValue.IbPathRecord.PathRecord.ComponentMask =
								IB_PATH_RECORD_COMP_NUMBPATH | IB_PATH_RECORD_COMP_SERVICEID |
								IB_PATH_RECORD_COMP_DGID | IB_PATH_RECORD_COMP_SGID;
							query.InputValue.IbPathRecord.PathRecord.PathRecord.NumbPath = 32;
						}

						status = omgt_query_sa(port, &query, &query_path_results);
						if ((status != FSUCCESS) || (query_path_results == NULL) || (query_path_results->ResultDataSize == 0)) {
							OMGT_OUTPUT_ERROR(port, "Error, failed to get path record (status=0x%x) query_path_results=%p: %s\n",
								(unsigned int)status, (void *)query_path_results, FSTATUS_MSG(status));
							status = FERROR;
							break;
						}
						else {
							PATH_RESULTS *path_record_results = (PATH_RESULTS *)query_path_results->QueryResult;
							OMGT_DBGPRINT(port, "Got Path Records: records=%d\n", path_record_results->NumPathRecords);
							if (path_record_results->NumPathRecords >= 1) {
							 // Find PathRecord with default P_Key or use PathRecords[0]
							for (ix = path_record_results->NumPathRecords - 1; ix > 0; ix--) {
								if ((path_record_results->PathRecords[ix].P_Key & 0x7FFF) ==
										(DEFAULT_P_KEY & 0x7FFF))
										break;
							}
							OMGT_DBGPRINT(port, "DGID in path record: 0x%lx:0x%lx\n",
								path_record_results->PathRecords[ix].DGID.Type.Global.SubnetPrefix,
								path_record_results->PathRecords[ix].DGID.Type.Global.InterfaceID);
								/* Check for Extended LID in the OPA GID */
								if ((path_record_results->PathRecords[ix].DGID.Type.Global.InterfaceID >> 40)
									== OMGT_STL_OUI) {
									port->primary_pm_lid =
										path_record_results->PathRecords[ix].DGID.Type.Global.InterfaceID & 0xFFFFFFFF;
								} else {
									port->primary_pm_lid = path_record_results->PathRecords[ix].DLID;
								}
								// Remember that for our own use
								port->primary_pm_sl = path_record_results->PathRecords[ix].u2.s.SL;
								OMGT_DBGPRINT(port, "Found Master PM LID 0x%x SL %u\n",
									port->primary_pm_lid, port->primary_pm_sl);
								} else {
									status = FERROR;
									OMGT_OUTPUT_ERROR(port, "Error, received no path records\n");
									break;
								}
						}
					}
				}
			}
			OMGT_DBGPRINT(port, "Number of PMs found %d\n", pmCount);
		}
	}
	// deallocate results buffers
	if (query_results)
		omgt_free_query_result_buffer(query_results);
	if (query_path_results)
		omgt_free_query_result_buffer(query_path_results);

	return status;
}

/**
 *  Get and Convert a MAD status code into string.
 *
 * @param port                    Local port to operate on. 
 * 
 * @return 
 *   The corresponding status string.
 */
const char* 
iba_pa_mad_status_msg(
    IN struct omgt_port              *port
    )
{
    // this is a little more complex than most due to bitfields and reserved
    // values
    MAD_STATUS  mad_status;
    uint16_t    code = port->pa_mad_status;

    mad_status.AsReg16 = port->pa_mad_status; // ignore reserved bits in mad_status
    if (code == MAD_STATUS_SUCCESS || (code & 0xff))
        return iba_mad_status_msg(mad_status);   // standard mad status fields
    else 
    {
		// SA specific status code field 0-6
		code = mad_status.S.ClassSpecific;
		if (code >= (unsigned)(sizeof(pa_sa_status_text)/sizeof(char*))) {
			// PA specific status code field - codes start at 'A' (10)
			code = mad_status.S.ClassSpecific - 9;
			if (code >= (unsigned)(sizeof(pa_status_text)/sizeof(char*)))
				return "Unknown PA Mad Status";
			else
				return pa_status_text[code];
		} else
			return pa_sa_status_text[code];
    }
}

/*
 * @brief Initialize PA connection on existing omgt_port session
 *
 * @param port                      omgt_port session to start PA interface on
 *
 * @return
 *  PACLIENT_OPERATIONAL - initialization successful
 *         PACLIENT_DOWN - initialization not successful/PaServer not available
 */
int
omgt_pa_service_connect(struct omgt_port *port)
{
	FSTATUS fstatus;
	uint8_t mclass = MCLASS_VFI_PM;
	struct omgt_class_args mgmt_class[2];
	int err = 0;
	IB_PORT_ATTRIBUTES *attrib = NULL;
	int pa_service_state = OMGT_SERVICE_STATE_DOWN;

	fstatus = omgt_get_portguid(port->hfi_num, port->hfi_port_num, NULL, port, NULL, NULL, NULL, &attrib, NULL, NULL, NULL, NULL, NULL);

	if (FSUCCESS == fstatus && attrib) {
		port->local_gid = attrib->GIDTable[0];
	} else {
		OMGT_OUTPUT_ERROR(port, "Could not get port guid: %s\n", iba_fstatus_msg(fstatus));
		goto done;
	}

	if ((fstatus = iba_pa_query_master_pm_lid(port)) != FSUCCESS) {
		OMGT_OUTPUT_ERROR(port, "Can't query primary PM LID!\n");
		if (fstatus == FUNAVAILABLE) {
			pa_service_state = OMGT_SERVICE_STATE_UNAVAILABLE;
		}
		goto done;
	}

	// register PA methods with OFED
	memset(mgmt_class, 0, sizeof(mgmt_class));
	mgmt_class[0].base_version = STL_BASE_VERSION;
	mgmt_class[0].mgmt_class = mclass;
	mgmt_class[0].class_version = STL_PM_CLASS_VERSION;
	mgmt_class[0].is_responding_client = 0;
	mgmt_class[0].is_trap_client = 0;
	mgmt_class[0].is_report_client = 0;
	mgmt_class[0].kernel_rmpp = 1;
	mgmt_class[0].use_methods = 0;
	mgmt_class[0].oui = ib_truescale_oui;

	if ((err = omgt_bind_classes(port, mgmt_class)) != 0) {
		OMGT_OUTPUT_ERROR(port, "Failed to  register management class 0x%02x: %s\n",
			mclass, strerror(err));
		goto done;
	}

	pa_service_state = OMGT_SERVICE_STATE_OPERATIONAL;
done:
	if (attrib)
		MemoryDeallocate(attrib);

	return (port->pa_service_state = pa_service_state);
}

/** 
 *  Get PM configuration data
 *
 * @param port          Port to operate on. 
 * @param pm_config     Pointer to PM config data to fill
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_pm_config(
    struct omgt_port     *port, 
    STL_PA_PM_CFG_DATA      *pm_config
    )
{
    OMGT_STATUS_T         fstatus = OMGT_STATUS_ERROR;
	STL_PA_PM_CFG_DATA * response;

    // Validate parameters
    if (NULL == port || !pm_config)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    OMGT_DBGPRINT(port, "Getting PM Configuration...\n");

    if ( ( response =
           iba_pa_single_mad_get_pm_config_response_query(port) ) != NULL )
    {
#if 0
TODO STL COMPILE!
        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port, "Interval = %u\n", response->sweepInterval);
        OMGT_DBGPRINT(port, "Max Clients = %u\n", response->MaxClients);
        OMGT_DBGPRINT(port, "History Size = %u\n", response->sizeHistory);
        OMGT_DBGPRINT(port, "Freeze Size = %u\n", response->sizeFreeze);
        OMGT_DBGPRINT(port, "Freeze Lease = %u\n", response->lease);
        OMGT_DBGPRINT(port, "Flags = 0x%X\n", response->pmFlags);
        OMGT_DBGPRINT(port,  "Memory Footprint = 0x%" PRIX64 "\n",
                  response->memoryFootprint );
#endif
        memcpy(pm_config, response, sizeof(*pm_config));

        MemoryDeallocate(response);
        fstatus = OMGT_STATUS_SUCCESS;
    }
    else
        OMGT_DBGPRINT(port, "Got NULL response - FAILED\n");

    return (fstatus);
}

/** 
 *  Get image info
 *
 * @param port              Port to operate on. 
 * @param pm_image_Id       Image ID of image info to get 
 * @param pm_image_info     Pointer to image info to fill 
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_image_info(
    struct omgt_port     *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id,
    STL_PA_IMAGE_INFO_DATA     *pm_image_info 
    )
{
    OMGT_STATUS_T         fstatus = OMGT_STATUS_ERROR;
    uint32          ix;
    STL_PA_IMAGE_INFO_DATA query = {{0}};
    STL_PA_IMAGE_INFO_DATA *response;;

    if (NULL == port || !pm_image_info)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    query.imageId = pm_image_id;

    OMGT_DBGPRINT(port, "Getting Image Info...\n");

    if ( ( response = iba_pa_multi_mad_get_image_info_response_query( 
       port, &query ) ) != NULL )
    {
#if 0        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port,  "RespImageNum = 0x%" PRIX64 " Offset = %d\n",
                  response->imageId.imageNumber, response->imageId.imageOffset );
        OMGT_DBGPRINT(port,  "sweepStart = 0x%" PRIX64 "\n",
                  response->sweepStart );
        OMGT_DBGPRINT(port,  "sweepDuration = %u\n",
                  response->sweepDuration );
        OMGT_DBGPRINT(port,  "numHFIPorts = %u\n",
                  response->numHFIPorts );
        OMGT_DBGPRINT(port,  "numSwNodes = %u\n",
                  response->numSwitchNodes );
        OMGT_DBGPRINT(port,  "numSwPorts = %u\n",
                  response->numSwitchPorts );
        OMGT_DBGPRINT(port,  "numLinks = %u\n",
                  response->numLinks );
        OMGT_DBGPRINT(port,  "numSMs = %u\n",
                  response->numSMs );
        OMGT_DBGPRINT(port,  "numFailedNodes = %u\n",
                  response->numFailedNodes );
        OMGT_DBGPRINT(port,  "numFailedPorts = %u\n",
                  response->numFailedPorts );
        OMGT_DBGPRINT(port,  "numSkippedPorts = %u\n",
                  response->numSkippedPorts );
        OMGT_DBGPRINT(port,  "numUnexpectedClearPorts = %u\n",
                  response->numUnexpectedClearPorts );
#endif
        memcpy(pm_image_info, response, sizeof(*pm_image_info));

        for (ix = pm_image_info->numSMs; ix < 2; ix++)
            memset(&pm_image_info->SMInfo[ix], 0, sizeof(STL_SMINFO_DATA));

        MemoryDeallocate(response);
        fstatus = OMGT_STATUS_SUCCESS;
    }
    else
        OMGT_DBGPRINT(port, "Got NULL response - FAILED\n");

    return (fstatus);
}

/** 
 *  Get list of group names
 *
 * @param port              Port to operate on. 
 * @param pm_group_list     Pointer to group list to fill 
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_group_list(
    struct omgt_port   *port,
    uint32             *pNum_Groups,
    STL_PA_GROUP_LIST  **pm_group_list
    )
{
    OMGT_STATUS_T				fstatus = OMGT_STATUS_ERROR;
    uint32						size_group_list;
    OMGT_QUERY					query;
    PQUERY_RESULT_VALUES		query_results = NULL;
    STL_PA_GROUP_LIST_RESULTS	*p;

    // Validate the parameters and state
    if (!port || !pNum_Groups || !pm_group_list || *pm_group_list)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));   // initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    OMGT_DBGPRINT(port, "Getting Multi Record Response For Group Data...\n");
    OMGT_DBGPRINT(port, "Query: Input=%s, Output=%s\n",
              iba_pa_query_input_type_msg(query.InputType),
              iba_pa_query_result_type_msg(query.OutputType) );

    fstatus = iba_pa_multi_mad_group_list_response_query( port, &query, &query_results );

    if (!query_results)
    {
        OMGT_DBGPRINT(port,  "PA Group List query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        OMGT_DBGPRINT(port,  "PA Group List query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status),
                  port->pa_mad_status, iba_pa_mad_status_msg(port) );
        goto fail;
    }

    else if (query_results->ResultDataSize == 0)
    {
        OMGT_DBGPRINT(port, "No Records Returned\n");
        *pNum_Groups = 0;
        fstatus = OMGT_STATUS_SUCCESS;
        goto done;
    }
    else
    {
		p = (STL_PA_GROUP_LIST_RESULTS*)query_results->QueryResult;

        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port,  "MadStatus 0x%X: %s\n", port->pa_mad_status,
                  iba_pa_mad_status_msg(port));
        OMGT_DBGPRINT(port, "%d Bytes Returned\n", query_results->ResultDataSize);
        OMGT_DBGPRINT(port, "PA Multiple MAD Response for Group Data:\n");
		OMGT_DBGPRINT(port, "NumGroupListRecords = %d\n",
				 (int)p->NumGroupListRecords );

		*pNum_Groups = p->NumGroupListRecords;
		size_group_list = sizeof(STL_PA_GROUP_LIST) * *pNum_Groups;

		if ( !( *pm_group_list = MemoryAllocate2AndClear( size_group_list, IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG ) ) )
        {
            OMGT_OUTPUT_ERROR(port, "can not allocate memory\n");
            goto fail;
        }

		memcpy(*pm_group_list, p->GroupListRecords, size_group_list);
        fstatus = OMGT_STATUS_SUCCESS;
    }

done:
    // omgt_query_port_fabric will have allocated a result buffer
    // we must free the buffer when we are done with it
    if (query_results)
        omgt_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = OMGT_STATUS_ERROR;
    goto done;

}

void omgt_pa_release_group_list(STL_PA_GROUP_LIST **pm_group_list)
{
	if (pm_group_list && *pm_group_list) {
		MemoryDeallocate(*pm_group_list);
		*pm_group_list = NULL;
	}
}
OMGT_STATUS_T
omgt_pa_get_group_list2(
    struct omgt_port     *port,
    STL_PA_IMAGE_ID_DATA pm_image_id_query,
    uint32               *pNum_Groups,
    STL_PA_GROUP_LIST2   **pm_group_list
    )
{
	OMGT_STATUS_T              fstatus = OMGT_STATUS_ERROR;
	uint32                     size_group_list;
	OMGT_QUERY                 query;
	PQUERY_RESULT_VALUES       query_results = NULL;
	STL_PA_IMAGE_ID_DATA       image_id = pm_image_id_query;
	STL_PA_GROUP_LIST2_RESULTS *p;

	// Validate the parameters and state
	if (!port || !pNum_Groups || !pm_group_list || *pm_group_list) {
		OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
		return (fstatus);
	}

	memset(&query, 0, sizeof(query));   // initialize reserved fields
	query.InputType     = InputTypeNoInput;
	query.OutputType    = OutputTypePaTableRecord;

	OMGT_DBGPRINT(port, "Getting Multi Record Response For Group List...\n");
	OMGT_DBGPRINT(port, "Query: Input=%s, Output=%s\n",
		iba_pa_query_input_type_msg(query.InputType),
		iba_pa_query_result_type_msg(query.OutputType));

	fstatus = iba_pa_multi_mad_group_list2_response_query(port, &query, &image_id, &query_results);

	if (!query_results) {
		OMGT_DBGPRINT(port,  "PA Group List2 query Failed: %s\n",
			iba_fstatus_msg(fstatus));
		goto fail;
	} else if (query_results->Status != FSUCCESS) {
		OMGT_DBGPRINT(port,  "PA Group List2 query Failed: %s MadStatus 0x%X: %s\n",
			iba_fstatus_msg(query_results->Status),
			port->pa_mad_status, iba_pa_mad_status_msg(port));
		goto fail;
	} else if (query_results->ResultDataSize == 0) {
		OMGT_DBGPRINT(port, "No Records Returned\n");
		*pNum_Groups = 0;
		fstatus = OMGT_STATUS_SUCCESS;
		goto done;
	} else {
		p = (STL_PA_GROUP_LIST2_RESULTS *)query_results->QueryResult;

		// TBD remove some of these after additional MAD testing
		OMGT_DBGPRINT(port, "MadStatus 0x%X: %s\n", port->pa_mad_status,
			iba_pa_mad_status_msg(port));
		OMGT_DBGPRINT(port, "%d Bytes Returned\n", query_results->ResultDataSize);
		OMGT_DBGPRINT(port, "PA Multiple MAD Response for Group Data:\n");
		OMGT_DBGPRINT(port, "NumGroupList2Records = %d\n",
			(int)p->NumGroupList2Records);

		*pNum_Groups = p->NumGroupList2Records;
		size_group_list = sizeof(STL_PA_GROUP_LIST2) * *pNum_Groups;

		if (!(*pm_group_list = MemoryAllocate2AndClear(size_group_list, IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG))) {
			OMGT_OUTPUT_ERROR(port, "can not allocate memory\n");
			goto fail;
		}
		memcpy(*pm_group_list, p->GroupList2Records, size_group_list);
		fstatus = OMGT_STATUS_SUCCESS;
	}

done:
	// omgt_query_port_fabric will have allocated a result buffer
	// we must free the buffer when we are done with it
	if (query_results)
		omgt_free_query_result_buffer(query_results);
	return (fstatus);
fail:
	fstatus = OMGT_STATUS_ERROR;
	goto done;
}

void omgt_pa_release_group_list2(STL_PA_GROUP_LIST2 **pm_group_list)
{
	if (pm_group_list && *pm_group_list) {
		MemoryDeallocate(*pm_group_list);
		*pm_group_list = NULL;
	}
}
/**
 *  Get list of vf names
 *
 * @param port              Port to operate on.
 * @param pm_vf_list     Pointer to vf list to fill
 *
 * @return
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_vf_list(
    struct omgt_port          *port,
	uint32					 *pNum_VFs,
    STL_PA_VF_LIST  		**pm_vf_list
    )
{
    OMGT_STATUS_T						fstatus = OMGT_STATUS_ERROR;
    uint32						size_vf_list;
    OMGT_QUERY					query;
    PQUERY_RESULT_VALUES		query_results = NULL;
    STL_PA_VF_LIST_RESULTS		*p;

    // Validate the parameters and state
    if (!port || !pNum_VFs || !pm_vf_list || *pm_vf_list)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));   // initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    OMGT_DBGPRINT(port, "Getting Multi Record Response For VF Data...\n");
    OMGT_DBGPRINT(port, "Query: Input=%s, Output=%s\n",
              iba_pa_query_input_type_msg(query.InputType),
              iba_pa_query_result_type_msg(query.OutputType) );

    fstatus = iba_pa_multi_mad_vf_list_response_query( port, &query, &query_results );

    if (!query_results)
    {
        OMGT_DBGPRINT(port,  "PA VF List query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        OMGT_DBGPRINT(port,  "PA VF List query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status),
                  port->pa_mad_status, iba_pa_mad_status_msg(port) );
        goto fail;
    }

    else if (query_results->ResultDataSize == 0)
    {
        OMGT_DBGPRINT(port, "No Records Returned\n");
        *pNum_VFs = 0;
        fstatus = OMGT_STATUS_SUCCESS;
        goto done;
    }
    else
    {
		p = (STL_PA_VF_LIST_RESULTS*)query_results->QueryResult;

        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port,  "MadStatus 0x%X: %s\n", port->pa_mad_status,
                  iba_pa_mad_status_msg(port));
        OMGT_DBGPRINT(port, "%d Bytes Returned\n", query_results->ResultDataSize);
        OMGT_DBGPRINT(port, "PA Multiple MAD Response for VF Data:\n");
		OMGT_DBGPRINT(port, "NumVFListRecords = %d\n",
				 (int)p->NumVFListRecords );

		*pNum_VFs = p->NumVFListRecords;
		size_vf_list = sizeof(STL_PA_VF_LIST) * *pNum_VFs;

		if ( !( *pm_vf_list = MemoryAllocate2AndClear( size_vf_list, IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG ) ) )
        {
            OMGT_OUTPUT_ERROR(port, "can not allocate memory\n");
            goto fail;
        }

		memcpy(*pm_vf_list, p->VFListRecords, size_vf_list);
        fstatus = OMGT_STATUS_SUCCESS;
    }

done:
    // omgt_query_port_fabric will have allocated a result buffer
    // we must free the buffer when we are done with it
    if (query_results)
        omgt_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = OMGT_STATUS_ERROR;
    goto done;

}

void omgt_pa_release_vf_list(STL_PA_VF_LIST **pm_vf_list)
{
	if (pm_vf_list && *pm_vf_list) {
		MemoryDeallocate(*pm_vf_list);
		*pm_vf_list = NULL;
	}
}
OMGT_STATUS_T
omgt_pa_get_vf_list2(
	struct omgt_port     *port,
	STL_PA_IMAGE_ID_DATA pm_image_id_query,
	uint32               *pNum_VFs,
	STL_PA_VF_LIST2      **pm_vf_list
	)
{
	OMGT_STATUS_T           fstatus = OMGT_STATUS_ERROR;
	uint32                  size_vf_list;
	OMGT_QUERY              query;
	PQUERY_RESULT_VALUES    query_results = NULL;
    STL_PA_IMAGE_ID_DATA    image_id = pm_image_id_query;
	STL_PA_VF_LIST2_RESULTS *p;

	// Validate the parameters and state
	if (!port || !pNum_VFs || !pm_vf_list || *pm_vf_list) {
		OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
		return (fstatus);
	}

	memset(&query, 0, sizeof(query));   // initialize reserved fields
	query.InputType     = InputTypeNoInput;
	query.OutputType    = OutputTypePaTableRecord;

	OMGT_DBGPRINT(port, "Getting Multi Record Response For VF Data...\n");
	OMGT_DBGPRINT(port, "Query: Input=%s, Output=%s\n",
		iba_pa_query_input_type_msg(query.InputType),
		iba_pa_query_result_type_msg(query.OutputType));

	fstatus = iba_pa_multi_mad_vf_list2_response_query(port, &query, &image_id, &query_results);
	if (!query_results) {
		OMGT_DBGPRINT(port, "PA VF List2 query Failed: %s\n",
			iba_fstatus_msg(fstatus));
		goto fail;
	} else if (query_results->Status != FSUCCESS) {
		OMGT_DBGPRINT(port, "PA VF List2 query Failed: %s MadStatus 0x%X: %s\n",
			iba_fstatus_msg(query_results->Status),
			port->pa_mad_status, iba_pa_mad_status_msg(port));
		goto fail;
	} else if (query_results->ResultDataSize == 0) {
		OMGT_DBGPRINT(port, "No Records Returned\n");
		*pNum_VFs = 0;
		fstatus = OMGT_STATUS_SUCCESS;
		goto done;
	} else {
		p = (STL_PA_VF_LIST2_RESULTS *)query_results->QueryResult;

		// TBD remove some of these after additional MAD testing
		OMGT_DBGPRINT(port, "MadStatus 0x%X: %s\n", port->pa_mad_status,
			iba_pa_mad_status_msg(port));
		OMGT_DBGPRINT(port, "%d Bytes Returned\n", query_results->ResultDataSize);
		OMGT_DBGPRINT(port, "PA Multiple MAD Response for VF Data:\n");
		OMGT_DBGPRINT(port, "NumVFList2Records = %d\n",
			(int)p->NumVFList2Records);

		*pNum_VFs = p->NumVFList2Records;
		size_vf_list = sizeof(STL_PA_VF_LIST2) * *pNum_VFs;

		if (!(*pm_vf_list = MemoryAllocate2AndClear(size_vf_list, IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG))) {
			OMGT_OUTPUT_ERROR(port, "can not allocate memory\n");
			goto fail;
		}

		memcpy(*pm_vf_list, p->VFList2Records, size_vf_list);
		fstatus = OMGT_STATUS_SUCCESS;
	}

done:
	// omgt_query_port_fabric will have allocated a result buffer
	// we must free the buffer when we are done with it
	if (query_results)
		omgt_free_query_result_buffer(query_results);
	return (fstatus);
fail:
	fstatus = OMGT_STATUS_ERROR;
	goto done;

}
void omgt_pa_release_vf_list2(STL_PA_VF_LIST2 **pm_vf_list)
{
	if (pm_vf_list && *pm_vf_list) {
		MemoryDeallocate(*pm_vf_list);
		*pm_vf_list = NULL;
	}
}
/** 
 *  Get group info
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of group info to get. 
 * @param group_name            Pointer to group name 
 * @param pm_image_id_resp      Pointer to image ID of group info returned. 
 * @param pm_group_info         Pointer to group info to fill. 
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_group_info(
    struct omgt_port     *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    char                *group_name, 
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp, 
    STL_PA_PM_GROUP_INFO_DATA     *pm_group_info
    )
{
    OMGT_STATUS_T                 fstatus = OMGT_STATUS_ERROR;
    STL_PA_IMAGE_ID_DATA           image_id = pm_image_id_query;
    OMGT_QUERY              query;
    PQUERY_RESULT_VALUES    query_results = NULL;
    STL_PA_GROUP_INFO_RESULTS   *p;

    // Validate the parameters and state
    if (!port || !group_name || !pm_group_info)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));	// initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    OMGT_DBGPRINT(port, "Getting Multi Record Response For Group Info...\n");
    OMGT_DBGPRINT(port, "Query: Input=%s, Output=%s\n",
             iba_pa_query_input_type_msg(query.InputType),
             iba_pa_query_result_type_msg(query.OutputType));

    fstatus = iba_pa_multi_mad_group_stats_response_query( 
       port, &query, group_name, &query_results, &image_id );

    if (!query_results)
    {
        OMGT_DBGPRINT(port,  "PA GroupInfo query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        OMGT_DBGPRINT(port,  "PA GroupInfo query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status), port->pa_mad_status,
                  iba_pa_mad_status_msg(port) );
        goto fail;
    }
    else if (query_results->ResultDataSize == 0)
    {
        OMGT_DBGPRINT(port, "No Records Returned\n");
        fstatus = OMGT_STATUS_SUCCESS;
        goto done;
    }
    else
    {
        p = (STL_PA_GROUP_INFO_RESULTS*)query_results->QueryResult;
#if 0
        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port,  "MadStatus 0x%X: %s\n", port->pa_mad_status,
        	iba_pa_mad_status_msg(port->pa_mad_status );
        OMGT_DBGPRINT(port, "%d Bytes Returned\n", query_results->ResultDataSize);
        OMGT_DBGPRINT(port, "PA Multiple MAD Response for Group Info group %s:\n",
                  group_name );
        OMGT_DBGPRINT(port, "NumGroupInfoRecords = %d\n",
                  (int)p->NumGroupInfoRecords );
        
        // TBD should this allow multiple records of group info ?
        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port, "\tgroup name: %s\n", p->GroupInfoRecords[0].groupName);
        OMGT_DBGPRINT(port, "\tinternal ports: %d\n", p->GroupInfoRecords[0].numInternalPorts);
        OMGT_DBGPRINT(port, "\tinternal rate min: %d\n", p->GroupInfoRecords[0].minInternalRate);
        OMGT_DBGPRINT(port, "\tinternal rate max: %d\n", p->GroupInfoRecords[0].maxInternalRate);
        OMGT_DBGPRINT(port, "\tinternal MBps max: %d\n", p->GroupInfoRecords[0].maxInternalMBps);
        OMGT_DBGPRINT(port, "\texternal ports: %d\n", p->GroupInfoRecords[0].numExternalPorts);
        OMGT_DBGPRINT(port, "\texternal rate min: %d\n", p->GroupInfoRecords[0].minExternalRate);
        OMGT_DBGPRINT(port, "\texternal rate max: %d\n", p->GroupInfoRecords[0].maxExternalRate);
        OMGT_DBGPRINT(port, "\texternal MBps max: %d\n", p->GroupInfoRecords[0].maxExternalMBps);
        OMGT_DBGPRINT(port,  "\tinternal util totalMBps: %" PRIu64 "\n",
                  p->GroupInfoRecords[0].internalUtilStats.totalMBps );
        OMGT_DBGPRINT(port,  "\tinternal util avgMBps: %u\n",
                  p->GroupInfoRecords[0].internalUtilStats.avgMBps );
        OMGT_DBGPRINT(port,  "\tinternal util maxMBps: %u\n",
                  p->GroupInfoRecords[0].internalUtilStats.maxMBps );
        OMGT_DBGPRINT(port,  "\tinternal util totalKPps: %" PRIu64 "\n",
                  p->GroupInfoRecords[0].internalUtilStats.totalKPps );
        OMGT_DBGPRINT(port,  "\tinternal util avgKPps: %u\n",
                  p->GroupInfoRecords[0].internalUtilStats.avgKPps );
        OMGT_DBGPRINT(port,  "\tinternal util maxKPps: %u\n",
                  p->GroupInfoRecords[0].internalUtilStats.maxKPps );
        OMGT_DBGPRINT(port,  "\tsend util stats: %" PRIu64 "\n",
                  p->GroupInfoRecords[0].sendUtilStats.totalMBps );
        OMGT_DBGPRINT(port,  "\trecv util stats: %" PRIu64 "\n",
                  p->GroupInfoRecords[0].recvUtilStats.totalMBps );
        OMGT_DBGPRINT(port,  "\tinternal max integrity errors: %u\n",
                  p->GroupInfoRecords[0].internalCategoryStats.categoryMaximums.integrityErrors );
        OMGT_DBGPRINT(port,  "\texternal max integrity errors: %u\n",
                  p->GroupInfoRecords[0].externalCategoryStats.categoryMaximums.integrityErrors );
        OMGT_DBGPRINT(port,  "\tinternal integrity bucket[4]: %u\n",
                  p->GroupInfoRecords[0].internalCategoryStats.ports[4].integrityErrors );
        OMGT_DBGPRINT(port,  "\texternal integrity bucket[4]: %u\n",
                  p->GroupInfoRecords[0].externalCategoryStats.ports[4].integrityErrors );
#endif
        memcpy(pm_group_info, p->GroupInfoRecords, sizeof(*pm_group_info));
        if (pm_image_id_resp)
            *pm_image_id_resp = p->GroupInfoRecords[0].imageId;

        fstatus = OMGT_STATUS_SUCCESS;
    }

done:
    // iba_pa_query_port_fabric_info will have allocated a result buffer
    // we must free the buffer when we are done with it
    if (query_results)
        omgt_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = OMGT_STATUS_ERROR;
    goto done;
}

/**
 *  Get vf info
 *
 * @param port                  Port to operate on.
 * @param pm_image_id_query     Image ID of vf info to get.
 * @param vf_name               Pointer to vf name
 * @param pm_image_id_resp      Pointer to image ID of vf info returned.
 * @param pm_vf_info            Pointer to vf info to fill.
 *
 * @return
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_vf_info(
    struct omgt_port           *port,
    STL_PA_IMAGE_ID_DATA        pm_image_id_query,
    char                       *vf_name,
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp,
    STL_PA_VF_INFO_DATA        *pm_vf_info
    )
{
	OMGT_STATUS_T status = OMGT_STATUS_ERROR;
	STL_PA_IMAGE_ID_DATA image_id = pm_image_id_query;
	OMGT_QUERY query;
	PQUERY_RESULT_VALUES query_results = NULL;
	STL_PA_VF_INFO_RESULTS *p;

	// Validate the parameters and state
	if (!port || !vf_name || !pm_vf_info){
		OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
		return status;
	}

	memset(&query, 0, sizeof(query));
	query.InputType  = InputTypeNoInput;
	query.OutputType = OutputTypePaTableRecord;

	OMGT_DBGPRINT(port, "Getting Multi Record Response for VF Info...\n");
	OMGT_DBGPRINT(port, "Query: Input=%s, Output=%s\n", iba_pa_query_input_type_msg(query.InputType),
	                                                    iba_pa_query_result_type_msg(query.OutputType));

	status = iba_pa_multi_mad_vf_info_response_query(port, &query, vf_name, &query_results, &image_id);

	if (!query_results)
	{
		OMGT_DBGPRINT(port, "PA VFInfo query Failed: %s\n", iba_fstatus_msg(status));
		goto fail;
	}
	else if (query_results->Status != FSUCCESS)
	{
		OMGT_DBGPRINT(port,  "PA VFInfo query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status), port->pa_mad_status,
                  iba_pa_mad_status_msg(port) );
		goto fail;
	}
	else if (query_results->ResultDataSize == 0)
	{
		OMGT_DBGPRINT(port, "No Records Returned\n");
		status = OMGT_STATUS_SUCCESS;
		goto done;
	}
	else
	{
		p = (STL_PA_VF_INFO_RESULTS*)query_results->QueryResult;

		memcpy(pm_vf_info, p->VFInfoRecords, sizeof(*pm_vf_info));
		if (pm_image_id_resp)
			*pm_image_id_resp = p->VFInfoRecords[0].imageId;

		status = OMGT_STATUS_SUCCESS;
	}

done:
	if (query_results)
		omgt_free_query_result_buffer(query_results);
	return (status);

fail:
	status = OMGT_STATUS_ERROR;
	goto done;
}

/**
 *  Get group config info
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of group config to get. 
 * @param group_name            Pointer to group name 
 * @param pm_image_id_resp      Pointer to image ID of group info returned. 
 * @param pm_group_config       Pointer to group config to fill. Upon successful return, a memory to 
 *                              contain the group config is allocated. The caller must call
 *                              omgt_pa_release_group_config to free the memory later.
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_group_config(
    struct omgt_port     *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    char                *group_name, 
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp, 
	uint32						*pNum_ports,
    STL_PA_PM_GROUP_CFG_RSP   **pm_group_config 
    )
{
    OMGT_STATUS_T           fstatus = OMGT_STATUS_ERROR;
    uint32                  num_ports;
    uint32                  size_group_config;
    STL_PA_IMAGE_ID_DATA           image_id = pm_image_id_query;
    OMGT_QUERY              query;
    PQUERY_RESULT_VALUES    query_results = NULL;
    STL_PA_GROUP_CONFIG_RESULTS *p;

    // Validate the parameters and state
    if (!port || !group_name || !pm_group_config || *pm_group_config)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));	// initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    OMGT_DBGPRINT(port, "Getting Group Config...\n");
    OMGT_DBGPRINT(port, "Query: Input=%s, Output=%s\n",
             iba_pa_query_input_type_msg(query.InputType),
             iba_pa_query_result_type_msg(query.OutputType));

    fstatus = iba_pa_multi_mad_group_config_response_query( 
       port, &query, group_name, &query_results, &image_id );

    if (!query_results)
    {
        OMGT_DBGPRINT(port,  "PA GroupConfig query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        OMGT_DBGPRINT(port,  "PA GroupConfig query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status), port->pa_mad_status,
                  iba_pa_mad_status_msg(port) );
        goto fail;
    }
    else if (query_results->ResultDataSize == 0)
    {
        OMGT_DBGPRINT(port, "No Records Returned\n");
        *pNum_ports = 0;
		fstatus = OMGT_STATUS_SUCCESS;
		goto done;
    }
    else
    {
        p = (STL_PA_GROUP_CONFIG_RESULTS*)query_results->QueryResult;
        
        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port, "MadStatus 0x%X: %s\n", port->pa_mad_status,
                  iba_pa_mad_status_msg(port) );
        OMGT_DBGPRINT(port, "%d Bytes Returned\n", query_results->ResultDataSize);
        OMGT_DBGPRINT(port, "PA Multiple MAD Response for GroupConfig group %s:\n",
                  group_name );
        OMGT_DBGPRINT(port, "NumGroupConfigRecords = %d\n",
                  (int)p->NumGroupConfigRecords );
        
        num_ports = p->NumGroupConfigRecords < MAX_PORTS_LIST ?
            p->NumGroupConfigRecords : MAX_PORTS_LIST;
		*pNum_ports = num_ports;
        size_group_config = sizeof(STL_PA_PM_GROUP_CFG_RSP) * num_ports;
        
        if ( !( *pm_group_config = MemoryAllocate2AndClear( size_group_config,
                                                            IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG ) ) )
        {
            OMGT_OUTPUT_ERROR(port, "can not allocate memory\n");
            goto fail;
        }
        
        OMGT_DBGPRINT(port,  "\tname:%s, ports:%u\n",
                  group_name,
                  (unsigned int)num_ports );
        
        memcpy(*pm_group_config, p->GroupConfigRecords, size_group_config); 
        fstatus = OMGT_STATUS_SUCCESS;
    }

done:
    // iba_pa_query_port_fabric_info will have allocated a result buffer
    // we must free the buffer when we are done with it
    omgt_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = OMGT_STATUS_ERROR;
    goto done;
}

/** 
 *  Release group config info
 *
 * @param port                  Port to operate on. 
 * @param pm_group_config       Pointer to pointer to the group config to free. 
 *
 * @return 
 *   None
 */
void 
omgt_pa_release_group_config(
    STL_PA_PM_GROUP_CFG_RSP   **pm_group_config
    )
{
    if (pm_group_config && *pm_group_config)
    {
        MemoryDeallocate(*pm_group_config);
        *pm_group_config = NULL;
    }

}

/**
 *  Get group node info
 *
 * @param port                  Port to operate on.
 * @param pm_image_id_query     Image ID of group config to get.
 * @param group_name            Pointer to group name
 * @param nodeLid               node LID
 * @param nodeGuid              node GUID
 * @param nodeDesc              node Description
 * @param pm_image_id_resp      Pointer to image ID of group info returned.
 * @param pm_group_config       Pointer to group config to fill. Upon successful return, a memory to
 *                              contain the group config is allocated. The caller must call
 *                              omgt_pa_release_group_config to free the memory later.
 *
 * @return
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_group_nodeinfo(
    struct omgt_port     	*port,
    STL_PA_IMAGE_ID_DATA       	pm_image_id_query,
    char                	*group_name,
    STL_LID             	nodeLid,
    uint64              	nodeGuid,
    char                        *nodeDesc,
    STL_PA_IMAGE_ID_DATA       	*pm_image_id_resp,
    uint32               	*pNum_nodes,
    STL_PA_GROUP_NODEINFO_RSP   **pm_group_nodeinfo
    )
{
    OMGT_STATUS_T           fstatus = OMGT_STATUS_ERROR;
    uint32                  num_nodes;
    uint32                  size_group_nodeinfo;
    STL_PA_IMAGE_ID_DATA    image_id = pm_image_id_query;
    OMGT_QUERY              query;
    PQUERY_RESULT_VALUES    query_results = NULL;
    STL_PA_GROUP_NODEINFO_RESULTS *p;

    // Validate the parameters and state
    if (!port || !group_name || !pm_group_nodeinfo || *pm_group_nodeinfo)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));	// initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    OMGT_DBGPRINT(port, "Getting Group Node Info...\n");
    OMGT_DBGPRINT(port, "Query: Input=%s, Output=%s\n",
             iba_pa_query_input_type_msg(query.InputType),
             iba_pa_query_result_type_msg(query.OutputType));

    fstatus = iba_pa_multi_mad_group_nodeinfo_response_query(
       port, &query, group_name, nodeLid, nodeGuid, nodeDesc, &query_results, &image_id );

    if (!query_results)
    {
        OMGT_DBGPRINT(port,  "PA GroupNodeInfo query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        OMGT_DBGPRINT(port,  "PA GroupNodeInfo query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status), port->pa_mad_status,
                  iba_pa_mad_status_msg(port) );
        goto fail;
    }
    else if (query_results->ResultDataSize == 0)
    {
        OMGT_DBGPRINT(port, "No Records Returned\n");
        *pNum_nodes = 0;
		fstatus = OMGT_STATUS_SUCCESS;
		goto done;
    }
    else
    {
        p = (STL_PA_GROUP_NODEINFO_RESULTS*)query_results->QueryResult;

        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port, "MadStatus 0x%X: %s\n", port->pa_mad_status,
                  iba_pa_mad_status_msg(port) );
        OMGT_DBGPRINT(port, "%d Bytes Returned\n", query_results->ResultDataSize);
        OMGT_DBGPRINT(port, "PA Multiple MAD Response for GroupNodeInfo group %s:\n",
                  group_name );
        OMGT_DBGPRINT(port, "NumGroupNodeInfoRecords = %d\n",
                  (int)p->NumGroupNodeInfoRecords );

        num_nodes = p->NumGroupNodeInfoRecords;
        *pNum_nodes = num_nodes;
        size_group_nodeinfo = sizeof(STL_PA_GROUP_NODEINFO_RSP) * num_nodes;

        if ( !( *pm_group_nodeinfo = MemoryAllocate2AndClear( size_group_nodeinfo,
                                                            IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG ) ) )
        {
            OMGT_OUTPUT_ERROR(port, "can not allocate memory\n");
            goto fail;
        }

        OMGT_DBGPRINT(port,  "\tname:%s, nodes:%u\n",
                  group_name,
                  (unsigned int)num_nodes );

        memcpy(*pm_group_nodeinfo, p->GroupNodeInfoRecords, size_group_nodeinfo);
        fstatus = OMGT_STATUS_SUCCESS;
    }

done:
    // iba_pa_query_port_fabric_info will have allocated a result buffer
    // we must free the buffer when we are done with it
    omgt_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = OMGT_STATUS_ERROR;
    goto done;
}

/**
 *  Release group node info
 *
 * @param pm_group_nodeinfo       Pointer to pointer to the group nodeinfo to free.
 *
 * @return
 *   None
 */
void
omgt_pa_release_group_nodeinfo(
    STL_PA_GROUP_NODEINFO_RSP   **pm_group_nodeinfo
    )
{
    if (pm_group_nodeinfo && *pm_group_nodeinfo)
    {
        MemoryDeallocate(*pm_group_nodeinfo);
        *pm_group_nodeinfo = NULL;
    }
}

/**
 *  Get group link info
 *
 * @param port                  Port to operate on.
 * @param pm_image_id_query     Image ID of group config to get.
 * @param group_name            Pointer to group name
 * @param inputLid              input LID
 * @param inputPort             input Port
 * @param pm_image_id_resp      Pointer to image ID of group info returned.
 * @param pm_group_config       Pointer to group config to fill. Upon successful return, a memory to
 *                              contain the group config is allocated. The caller must call
 *                              omgt_pa_release_group_config to free the memory later.
 *
 * @return
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_group_linkinfo(
    struct omgt_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    char                *group_name,
    STL_LID             inputLid,
    uint8               inputPort,
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp,
    uint32               *pNum_links,
    STL_PA_GROUP_LINKINFO_RSP   **pm_group_linkinfo
    )
{
    OMGT_STATUS_T           fstatus = OMGT_STATUS_ERROR;
    uint32                  num_links;
    uint32                  size_group_linkinfo;
    STL_PA_IMAGE_ID_DATA    image_id = pm_image_id_query;
    OMGT_QUERY              query;
    PQUERY_RESULT_VALUES    query_results = NULL;
    STL_PA_GROUP_LINKINFO_RESULTS *p;

    // Validate the parameters and state
    if (!port || !group_name || !pm_group_linkinfo || *pm_group_linkinfo)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));	// initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    OMGT_DBGPRINT(port, "Getting Group Link Info...\n");
    OMGT_DBGPRINT(port, "Query: Input=%s, Output=%s\n",
             iba_pa_query_input_type_msg(query.InputType),
             iba_pa_query_result_type_msg(query.OutputType));

    fstatus = iba_pa_multi_mad_group_linkinfo_response_query(
       port, &query, group_name, inputLid, inputPort, &query_results, &image_id );

    if (!query_results)
    {
        OMGT_DBGPRINT(port,  "PA GroupLinkInfo query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        OMGT_DBGPRINT(port,  "PA GroupLinkInfo query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status), port->pa_mad_status,
                  iba_pa_mad_status_msg(port) );
        goto fail;
    }
    else if (query_results->ResultDataSize == 0)
    {
        OMGT_DBGPRINT(port, "No Records Returned\n");
        *pNum_links = 0;
		fstatus = OMGT_STATUS_SUCCESS;
		goto done;
    }
    else
    {
        p = (STL_PA_GROUP_LINKINFO_RESULTS*)query_results->QueryResult;

        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port, "MadStatus 0x%X: %s\n", port->pa_mad_status,
                  iba_pa_mad_status_msg(port) );
        OMGT_DBGPRINT(port, "%d Bytes Returned\n", query_results->ResultDataSize);
        OMGT_DBGPRINT(port, "PA Multiple MAD Response for GroupLinkInfo group %s:\n",
                  group_name );
        OMGT_DBGPRINT(port, "NumGroupLinkInfoRecords = %d\n",
                  (int)p->NumGroupLinkInfoRecords );

        num_links = p->NumGroupLinkInfoRecords;
        *pNum_links = num_links;
        size_group_linkinfo = sizeof(STL_PA_GROUP_LINKINFO_RSP) * num_links;

        if ( !( *pm_group_linkinfo = MemoryAllocate2AndClear( size_group_linkinfo,
                                                            IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG ) ) )
        {
            OMGT_OUTPUT_ERROR(port, "can not allocate memory\n");
            goto fail;
        }

        OMGT_DBGPRINT(port,  "\tname:%s, links:%u\n",
                  group_name,
                  (unsigned int)num_links );

        memcpy(*pm_group_linkinfo, p->GroupLinkInfoRecords, size_group_linkinfo);
        fstatus = OMGT_STATUS_SUCCESS;
    }

done:
    // iba_pa_query_port_fabric_info will have allocated a result buffer
    // we must free the buffer when we are done with it
    omgt_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = OMGT_STATUS_ERROR;
    goto done;
}

/**
 *  Release group link info
 *
 * @param port         		Port to operate on.
 * @param pm_group_nodeinfo     Pointer to pointer to the group nodeinfo to free.
 *
 * @return
 *   None
 */
void
omgt_pa_release_group_linkinfo(
    STL_PA_GROUP_LINKINFO_RSP   **pm_group_linkinfo
    )
{
    if (pm_group_linkinfo && *pm_group_linkinfo)
    {
        MemoryDeallocate(*pm_group_linkinfo);
        *pm_group_linkinfo = NULL;
    }
}

/* Get VF config info
 *
 * @param port					Port to operate on
 * @param pm_image_id_query		Image ID of VF config to get.
 * @param vf_name				Pointer to VF name.
 * @param pm_image_id_resp		Pointer to image ID of VF info returned.
 * @param pm_vf_config			Pointer to VF config to fill. Upon successful return, a memory to
 * 								contain the VF config is allocated. The caller must call
 * 								omgt_pa_release_vf_config to free the memory later.
 *
 * 	@return
 * 	  OMGT_STATUS_SUCCESS - Get successful
 * 	    OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_vf_config(
	struct omgt_port				*port,
	STL_PA_IMAGE_ID_DATA		pm_image_id_query,
	char						*vf_name,
	STL_PA_IMAGE_ID_DATA		*pm_image_id_resp,
	uint32						*pNum_ports,
	STL_PA_VF_CFG_RSP		**pm_vf_config
	)
{
    OMGT_STATUS_T           fstatus = OMGT_STATUS_ERROR;
    uint32                  num_ports;
    uint32                  size_vf_config;
    STL_PA_IMAGE_ID_DATA    image_id = pm_image_id_query;
    OMGT_QUERY              query;
    PQUERY_RESULT_VALUES    query_results = NULL;
    STL_PA_VF_CONFIG_RESULTS *p;

    // Validate the parameters and state
    if (!port || !vf_name || !pm_vf_config || *pm_vf_config)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));	// initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    OMGT_DBGPRINT(port, "Getting Group Config...\n");
    OMGT_DBGPRINT(port, "Query: Input=%s, Output=%s\n",
             iba_pa_query_input_type_msg(query.InputType),
             iba_pa_query_result_type_msg(query.OutputType));

    fstatus = iba_pa_multi_mad_vf_config_response_query( 
       port, &query, vf_name, &query_results, &image_id );

    if (!query_results)
    {
        OMGT_DBGPRINT(port,  "PA VFConfig query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        OMGT_DBGPRINT(port,  "PA VFConfig query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status), port->pa_mad_status,
                  iba_pa_mad_status_msg(port) );
        goto fail;
    }
    else if (query_results->ResultDataSize == 0)
    {
        OMGT_DBGPRINT(port, "No Records Returned\n");
        *pNum_ports = 0;
		fstatus = OMGT_STATUS_SUCCESS;
		goto done;
    }
    else
    {
        p = (STL_PA_VF_CONFIG_RESULTS*)query_results->QueryResult;
        
        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port, "MadStatus 0x%X: %s\n", port->pa_mad_status,
                  iba_pa_mad_status_msg(port) );
        OMGT_DBGPRINT(port, "%d Bytes Returned\n", query_results->ResultDataSize);
        OMGT_DBGPRINT(port, "PA Multiple MAD Response for VFConfig group %s:\n",
                  vf_name );
        OMGT_DBGPRINT(port, "NumVFConfigRecords = %d\n",
                  (int)p->NumVFConfigRecords );
        
        num_ports = p->NumVFConfigRecords < MAX_PORTS_LIST ?
            p->NumVFConfigRecords : MAX_PORTS_LIST;
		*pNum_ports = num_ports;
        size_vf_config = sizeof(STL_PA_VF_CFG_RSP) * num_ports;
        
        if ( !( *pm_vf_config = MemoryAllocate2AndClear( size_vf_config,
                                                            IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG ) ) )
        {
            OMGT_OUTPUT_ERROR(port, "can not allocate memory\n");
            goto fail;
        }
        
        OMGT_DBGPRINT(port, "\tname:%s, ports:%u\n",
                  vf_name,
                  (unsigned int)num_ports );
        
        memcpy(*pm_vf_config, p->VFConfigRecords, size_vf_config); 
        fstatus = OMGT_STATUS_SUCCESS;
    }

done:
    // iba_pa_query_port_fabric_info will have allocated a result buffer
    // we must free the buffer when we are done with it
    omgt_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = OMGT_STATUS_ERROR;
    goto done;
}


/* Release VF config info
 *
 * @param port					Port to operate on.
 * @param pm_vf_config			Pointer to pointer to the VF config to free
 *
 * @return
 *   None
 */
void
omgt_pa_release_vf_config(
	STL_PA_VF_CFG_RSP		**pm_vf_config
	)
{
    if (pm_vf_config && *pm_vf_config)
    {
        MemoryDeallocate(*pm_vf_config);
        *pm_vf_config = NULL;
    }
}

/**
 *  Get group focus portlist
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of group focus portlist to get. 
 * @param group_name            Pointer to group name.
 * @param select                Select value for focus portlist. 
 * @param start                 Start index value of portlist 
 * @param range                 Index range of portlist. 
 * @param pm_image_id_resp      Pointer to image ID of group focus portlist returned. 
 * @param pm_group_focus        Pointer to pointer to focus portlist to fill. Upon 
 *                              successful return, a memory to contain the group focus
 *                              portlist is allocated. The caller must call
 *                              omgt_pa_release_group_focus to free the memory later.
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */

OMGT_STATUS_T
omgt_pa_get_group_focus(
    struct omgt_port     *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    char                *group_name, 
    uint32              select, 
    uint32              start, 
    uint32              range,
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp, 
	uint32						*pNum_ports,
    STL_FOCUS_PORTS_RSP    **pm_group_focus 
    )
{
    OMGT_STATUS_T           fstatus = OMGT_STATUS_ERROR;
    uint32                  num_ports;
    uint32                  size_group_focus;
    STL_PA_IMAGE_ID_DATA           image_id = pm_image_id_query;
    OMGT_QUERY              query;
    PQUERY_RESULT_VALUES    query_results = NULL;
    STL_PA_FOCUS_PORTS_RESULTS  *p;

    // Validate the parameters and state
    if (!port ||
        !group_name ||
        (range > MAX_PORTS_LIST) ||
        !pm_group_focus || *pm_group_focus)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));   // initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    OMGT_DBGPRINT(port, "Getting Group Focus...\n");
    OMGT_DBGPRINT(port, "Query: Input=%s, Output=%s\n",
             iba_pa_query_input_type_msg(query.InputType),
             iba_pa_query_result_type_msg(query.OutputType));

    fstatus = iba_pa_multi_mad_focus_ports_response_query(port, &query,
		group_name, select, start, range, &query_results, &image_id );

    if (!query_results)
    {
        OMGT_DBGPRINT(port, "PA Group Focus query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        OMGT_DBGPRINT(port, "PA Group Focus query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status), port->pa_mad_status,
                  iba_pa_mad_status_msg(port) );
        goto fail;
    }
    else if (query_results->ResultDataSize == 0)
    {
        OMGT_DBGPRINT(port, "No Records Returned\n");
        *pNum_ports = 0;
		fstatus = OMGT_STATUS_SUCCESS;
		goto done;
    }
    else
    {
        p = (STL_PA_FOCUS_PORTS_RESULTS*)query_results->QueryResult;
        
        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port, "MadStatus 0x%X: %s\n", port->pa_mad_status,
                  iba_pa_mad_status_msg(port) );
        OMGT_DBGPRINT(port, "%d Bytes Returned\n", query_results->ResultDataSize);
        OMGT_DBGPRINT(port, "PA Multiple MAD Response for Focus portlist group %s:\n",
                 group_name );
        OMGT_DBGPRINT(port, "NumFocusPortsRecords = %d\n",
                  (int)p->NumFocusPortsRecords );
        
        num_ports = p->NumFocusPortsRecords < range ?
            p->NumFocusPortsRecords : range;

		*pNum_ports = num_ports;
        size_group_focus = sizeof(STL_FOCUS_PORTS_RSP) * num_ports;
        
        if ( !( *pm_group_focus = MemoryAllocate2AndClear( size_group_focus,
                                                         IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG ) ) )
        {
            OMGT_OUTPUT_ERROR(port, "can not allocate memory\n");
            goto fail;
        }
        
        OMGT_DBGPRINT(port,  "\tname:%s, ports:%u\n",
                  group_name,
                  (unsigned int)num_ports );
        
        memcpy(*pm_group_focus, p->FocusPortsRecords, size_group_focus); 
        fstatus = OMGT_STATUS_SUCCESS;
    }

done:
    // iba_pa_query_port_fabric_info will have allocated a result buffer
    // we must free the buffer when we are done with it
    omgt_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = OMGT_STATUS_ERROR;
    goto done;
}

/** 
 *  Release group focus portlist
 *
 * @param port                  Port to operate on. 
 * @param pm_group_config       Pointer to pointer to the group focus portlist to free. 
 *
 * @return 
 *   None
 */
void 
omgt_pa_release_group_focus(
    STL_FOCUS_PORTS_RSP     **pm_group_focus
   )
{
    if (pm_group_focus && *pm_group_focus)
    {
        MemoryDeallocate(*pm_group_focus);
        *pm_group_focus = NULL;
    }
}


/*
 *  Get VF focus portlist
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of vf focus portlist to get. 
 * @param vf_name            Pointer to vf name.
 * @param select                Select value for focus portlist. 
 * @param start                 Start index value of portlist 
 * @param range                 Index range of portlist. 
 * @param pm_image_id_resp      Pointer to image ID of group focus portlist returned. 
 * @param pm_vf_focus        Pointer to pointer to focus portlist to fill. Upon 
 *                              successful return, a memory to contain the group focus
 *                              portlist is allocated. The caller must call
 *                              omgt_pa_release_vf_focus to free the memory later.
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */

OMGT_STATUS_T
omgt_pa_get_vf_focus(
    struct omgt_port           *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    char                      *vf_name, 
    uint32                     select, 
    uint32                     start, 
    uint32                     range,
    STL_PA_IMAGE_ID_DATA      *pm_image_id_resp, 
	uint32						*pNum_ports,
    STL_PA_VF_FOCUS_PORTS_RSP     **pm_vf_focus 
    )
{
    OMGT_STATUS_T           fstatus = OMGT_STATUS_ERROR;
    uint32                  num_ports;
    uint32                  size_vf_focus;
    STL_PA_IMAGE_ID_DATA           image_id = pm_image_id_query;
    OMGT_QUERY              query;
    PQUERY_RESULT_VALUES    query_results = NULL;
    STL_PA_VF_FOCUS_PORTS_RESULTS  *p;

    // Validate the parameters and state
    if (!port ||
        !vf_name ||
        (range > MAX_PORTS_LIST) ||
        !pm_vf_focus || *pm_vf_focus)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));   // initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    OMGT_DBGPRINT(port, "Getting VF Focus...\n");
    OMGT_DBGPRINT(port, "Query: Input=%s, Output=%s\n",
             iba_pa_query_input_type_msg(query.InputType),
             iba_pa_query_result_type_msg(query.OutputType));

	fstatus = iba_pa_multi_mad_vf_focus_ports_response_query(port, &query,
		vf_name, select, start, range, &query_results, &image_id);

    if (!query_results)
    {
        OMGT_DBGPRINT(port, "PA VF Focus query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        OMGT_DBGPRINT(port,  "PA VF Focus query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status), port->pa_mad_status,
                  iba_pa_mad_status_msg(port) );
        goto fail;
    }
    else if (query_results->ResultDataSize == 0)
    {
        OMGT_DBGPRINT(port, "No Records Returned\n");
        *pNum_ports = 0;
		fstatus = OMGT_STATUS_SUCCESS;
		goto done;
    }
    else
    {
        p = (STL_PA_VF_FOCUS_PORTS_RESULTS*)query_results->QueryResult;
        
        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port, " MadStatus 0x%X: %s\n", port->pa_mad_status,
                  iba_pa_mad_status_msg(port) );
        OMGT_DBGPRINT(port, "%d Bytes Returned\n", query_results->ResultDataSize);
        OMGT_DBGPRINT(port, "PA Multiple MAD Response for Focus portlist vf %s:\n",
                 vf_name );
        OMGT_DBGPRINT(port, "NumVFFocusPortsRecords = %d\n",
                  (int)p->NumVFFocusPortsRecords );
        
        num_ports = p->NumVFFocusPortsRecords < range ?
            p->NumVFFocusPortsRecords : range;
		*pNum_ports = num_ports;
        size_vf_focus = sizeof(STL_PA_VF_FOCUS_PORTS_RSP) * num_ports;
        
        if ( !( *pm_vf_focus = MemoryAllocate2AndClear( size_vf_focus,
                                                         IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG ) ) )
        {
            OMGT_OUTPUT_ERROR(port, "can not allocate memory\n");
            goto fail;
        }
        
        OMGT_DBGPRINT(port,  "\tname:%s, ports:%u\n",
                  vf_name,
                  (unsigned int)num_ports );
        
        memcpy(*pm_vf_focus, p->FocusPortsRecords, size_vf_focus); 
        fstatus = OMGT_STATUS_SUCCESS;
    }

done:
    // iba_pa_query_port_fabric_info will have allocated a result buffer
    // we must free the buffer when we are done with it
    omgt_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = OMGT_STATUS_ERROR;
    goto done;
}


/* 
 *  Release vf focus portlist
 *
 * @param port                  Port to operate on. 
 * @param pm_vf_focus       Pointer to pointer to the vf focus portlist to free. 
 *
 * @return 
 *   None
 */
void 
omgt_pa_release_vf_focus(
    STL_PA_VF_FOCUS_PORTS_RSP     **pm_vf_focus
   )
{
    if (pm_vf_focus && *pm_vf_focus)
    {
        MemoryDeallocate(*pm_vf_focus);
        *pm_vf_focus = NULL;
    }
}

/**
 *  Get port statistics (counters). Supports 32 bit lids
 *
 * @param port                  Port to operate on.
 * @param pm_image_id_query     Image ID of port counters to get.
 * @param lid                   LID of node.
 * @param port_num              Port number.
 * @param pm_image_id_resp      Pointer to image ID of port counters returned.
 * @param port_counters         Pointer to port counters to fill.
 * @param flags                 Pointer to flags
 * @param delta                 1 for delta counters, 0 for raw image counters.
 * @param user_cntrs            1 for running counters, 0 for image counters. (delta must be 0)
 *
 * @return
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_port_stats2(
    struct omgt_port         *port,
    STL_PA_IMAGE_ID_DATA     pm_image_id_query,
    STL_LID                  lid,
    uint8                    port_num,
    STL_PA_IMAGE_ID_DATA    *pm_image_id_resp,
    STL_PORT_COUNTERS_DATA  *port_counters,
    uint32                  *flags,
    uint32                   delta,
	uint32                   user_cntrs
    )
{
    OMGT_STATUS_T            fstatus = OMGT_STATUS_ERROR;
    STL_PA_IMAGE_ID_DATA     image_id = pm_image_id_query;
    STL_PORT_COUNTERS_DATA  *response;

    // Validate the parameters and state
    if (!port || !port_counters)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    OMGT_DBGPRINT(port, "Getting Port Counters...\n");

    if ( ( response = iba_pa_single_mad_port_counters_response_query(
       port, lid, port_num, delta, user_cntrs, &image_id ) ) != NULL )
    {
        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port, "%s Controlled Port Counters (%s) Response for nodeLid 0x%X portNumber %d%s%s:\n",
				 (user_cntrs?"User":"PM"), (delta?"Delta":"Total"), lid, port_num,
				 response->flags & STL_PA_PC_FLAG_UNEXPECTED_CLEAR?" (Unexpected Clear)":"",
				 response->flags & STL_PA_PC_FLAG_CLEAR_FAIL?" (Clear Unsuccessful)":"" );
        OMGT_DBGPRINT(port, "\tXmitData = %" PRIu64 "\n", response->portXmitData);
        OMGT_DBGPRINT(port, "\tRcvData = %" PRIu64 "\n", response->portRcvData);
        OMGT_DBGPRINT(port, "\tXmitPkts = %" PRIu64 "\n", response->portXmitPkts);
        OMGT_DBGPRINT(port, "\tRcvPkts = %" PRIu64 "\n", response->portRcvPkts);
        OMGT_DBGPRINT(port, "\tMulticastXmitPkts = %" PRIu64 "\n", response->portMulticastXmitPkts);
        OMGT_DBGPRINT(port, "\tMulticastRcvPkts = %" PRIu64 "\n", response->portMulticastRcvPkts);

        OMGT_DBGPRINT(port, "\tLinkQualityIndicator = %u\n", response->lq.s.linkQualityIndicator);
        OMGT_DBGPRINT(port, "\tUncorrectableErrors = %u\n", response->uncorrectableErrors); // 8 bit
        OMGT_DBGPRINT(port, "\tLinkDowned = %u\n", response->linkDowned);	// 32 bit
        OMGT_DBGPRINT(port, "\tNumLanesDown = %u\n", response->lq.s.numLanesDown);
        OMGT_DBGPRINT(port, "\tRcvErrors = %"PRIu64"\n", response->portRcvErrors);
        OMGT_DBGPRINT(port,  "\tExcessiveBufferOverruns = %"PRIu64"\n",
                  response->excessiveBufferOverruns );
        OMGT_DBGPRINT(port, "\tFMConfigErrors = %"PRIu64"\n",
                  response->fmConfigErrors );
        OMGT_DBGPRINT(port,  "\tLinkErrorRecovery = %u\n", response->linkErrorRecovery);	// 32 bit
        OMGT_DBGPRINT(port, "\tLocalLinkIntegrityErrors = %"PRIu64"\n",
                  response->localLinkIntegrityErrors );
        OMGT_DBGPRINT(port, "\tRcvRemotePhysicalErrors = %"PRIu64"\n",
                  response->portRcvRemotePhysicalErrors);
        OMGT_DBGPRINT(port, "\tXmitConstraintErrors = %"PRIu64"\n",
                  response->portXmitConstraintErrors );
        OMGT_DBGPRINT(port,  "\tRcvConstraintErrors = %"PRIu64"\n",
                  response->portRcvConstraintErrors );
        OMGT_DBGPRINT(port, "\tRcvSwitchRelayErrors = %"PRIu64"\n",
                  response->portRcvSwitchRelayErrors);
        OMGT_DBGPRINT(port, "\tXmitDiscards = %"PRIu64"\n", response->portXmitDiscards);
        OMGT_DBGPRINT(port,  "\tCongDiscards = %"PRIu64"\n",
                  response->swPortCongestion);
        OMGT_DBGPRINT(port, "\tRcvFECN = %"PRIu64"\n", response->portRcvFECN);
        OMGT_DBGPRINT(port, "\tRcvBECN = %"PRIu64"\n", response->portRcvBECN);
        OMGT_DBGPRINT(port, "\tMarkFECN = %"PRIu64"\n", response->portMarkFECN);
        OMGT_DBGPRINT(port, "\tXmitTimeCong = %"PRIu64"\n", response->portXmitTimeCong);
        OMGT_DBGPRINT(port, "\tXmitWait = %"PRIu64"\n", response->portXmitWait);
        OMGT_DBGPRINT(port, "\tXmitWastedBW = %"PRIu64"\n", response->portXmitWastedBW);
        OMGT_DBGPRINT(port, "\tXmitWaitData = %"PRIu64"\n", response->portXmitWaitData);
        OMGT_DBGPRINT(port, "\tRcvBubble = %"PRIu64"\n", response->portRcvBubble);
        if (pm_image_id_resp)
            *pm_image_id_resp = response->imageId;
        if (flags)
            *flags = response->flags;

        memcpy(port_counters, response, sizeof(*port_counters));
        MemoryDeallocate(response);
        fstatus = OMGT_STATUS_SUCCESS;
    }
    else
        OMGT_DBGPRINT(port, "Got NULL response - FAILED\n");

    return (fstatus);
}

/* Deprecated. Use omgt_get_vf_port_stats2
*  This Should be removed in favor of the version that supports 32bit lids
*  when opamgt public library decides breaking compatibility is acceptable. 
*/
OMGT_STATUS_T
omgt_pa_get_port_stats(
    struct omgt_port         *port,
    STL_PA_IMAGE_ID_DATA     pm_image_id_query,
    uint16_t                 lid,
    uint8                    port_num,
    STL_PA_IMAGE_ID_DATA    *pm_image_id_resp,
    STL_PORT_COUNTERS_DATA  *port_counters,
    uint32                  *flags,
    uint32                   delta,
	uint32                   user_cntrs
    )
{
	OMGT_DBGPRINT(port, "omgt_pa_get_port_stats is deprecated. Please use omgt_pa_get_port_stats2\n");
	return omgt_pa_get_port_stats2(port, pm_image_id_query, lid, port_num, pm_image_id_resp, port_counters, flags, delta, user_cntrs);
}



/**
 *  Clear specified port counters for specified port
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of port counters to clear. 
 * @param lid                   LID of port.
 * @param port_num              Port number. 
 * @param selct_flags           Port's counters to clear. 
 *
 * @return 
 *   FSUCCESS - Clear successful
 *     FERROR - Error
 */
FSTATUS 
pa_client_clr_port_counters( 
    struct omgt_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query, 
    STL_LID             lid,
    uint8               port_num, 
    uint32              select_flag 
    )
{
    FSTATUS                 fstatus = FERROR;
    STL_CLR_PORT_COUNTERS_DATA  *response;
    
    // Validate the parameters and state
    if (!port)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }
  
    if ( ( response =
           iba_pa_single_mad_clr_port_counters_response_query(
               port, lid, port_num, select_flag ) ) )
    {
        MemoryDeallocate(response);
        fstatus = FSUCCESS;
    }

    return (fstatus);
}

/**
 *  Get vf port statistics (counters). Supports 32 bit lids. 
 *
 * @param port                  Port to operate on.
 * @param pm_image_id_query     Image ID of port counters to get.
 * @param vf_name				Pointer to VF name.
 * @param lid                   LID of node.
 * @param port_num              Port number.
 * @param pm_image_id_resp      Pointer to image ID of port counters returned.
 * @param vf_port_counters      Pointer to vf port counters to fill.
 * @param flags                 Pointer to flags
 * @param delta                 1 for delta counters, 0 for raw image counters.
 * @param user_cntrs            1 for running counters, 0 for image counters. (delta must be 0)
 *
 * @return
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_vf_port_stats2(
    struct omgt_port              *port,
    STL_PA_IMAGE_ID_DATA          pm_image_id_query,
	char				         *vf_name,
    STL_LID                       lid,
    uint8                         port_num,
    STL_PA_IMAGE_ID_DATA         *pm_image_id_resp,
    STL_PA_VF_PORT_COUNTERS_DATA *vf_port_counters,
    uint32                       *flags,
    uint32                        delta,
	uint32                        user_cntrs
    )
{
	OMGT_STATUS_T                 fstatus = OMGT_STATUS_ERROR;
    STL_PA_IMAGE_ID_DATA          image_id = pm_image_id_query;
    STL_PA_VF_PORT_COUNTERS_DATA *response;

    // Validate the parameters and state
    if (!port || !vf_port_counters)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    OMGT_DBGPRINT(port, "Getting Port Counters...\n");

    if ( ( response = iba_pa_single_mad_vf_port_counters_response_query(
       port, lid, port_num, delta, user_cntrs, vf_name, &image_id ) ) != NULL )
    {
        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port, "%s Controlled VF Port Counters (%s) Response for nodeLid 0x%X portNumber %d%s%s:\n",
				 (user_cntrs?"User":"PM"), (delta?"Delta":"Total"), lid, port_num,
				 response->flags & STL_PA_PC_FLAG_UNEXPECTED_CLEAR?" (Unexpected Clear)":"",
				 response->flags & STL_PA_PC_FLAG_CLEAR_FAIL?" (Clear Unsuccessful)":"" );
        OMGT_DBGPRINT(port, "\tvfName = %s\n", response->vfName);
        OMGT_DBGPRINT(port, "\tXmitData = %" PRIu64 "\n", response->portVFXmitData);
        OMGT_DBGPRINT(port, "\tRcvData = %" PRIu64 "\n", response->portVFRcvData);
        OMGT_DBGPRINT(port, "\tXmitPkts = %" PRIu64 "\n", response->portVFXmitPkts);
        OMGT_DBGPRINT(port, "\tRcvPkts = %" PRIu64 "\n", response->portVFRcvPkts);
        OMGT_DBGPRINT(port, "\tXmitDiscards = %"PRIu64"\n", response->portVFXmitDiscards);
        OMGT_DBGPRINT(port, "\tCongDiscards = %"PRIu64"\n", response->swPortVFCongestion);
        OMGT_DBGPRINT(port, "\tRcvFECN = %"PRIu64"\n", response->portVFRcvFECN);
        OMGT_DBGPRINT(port, "\tRcvBECN = %"PRIu64"\n", response->portVFRcvBECN);
        OMGT_DBGPRINT(port, "\tMarkFECN = %"PRIu64"\n", response->portVFMarkFECN);
        OMGT_DBGPRINT(port, "\tXmitTimeCong = %"PRIu64"\n", response->portVFXmitTimeCong);
        OMGT_DBGPRINT(port, "\tXmitWait = %"PRIu64"\n", response->portVFXmitWait);
        OMGT_DBGPRINT(port, "\tXmitWastedBW = %"PRIu64"\n", response->portVFXmitWastedBW);
        OMGT_DBGPRINT(port, "\tXmitWaitData = %"PRIu64"\n", response->portVFXmitWaitData);
        OMGT_DBGPRINT(port, "\tRcvBubble = %"PRIu64"\n", response->portVFRcvBubble);
        if (pm_image_id_resp)
            *pm_image_id_resp = response->imageId;
        if (flags)
            *flags = response->flags;

        memcpy(vf_port_counters, response, sizeof(*vf_port_counters));
        MemoryDeallocate(response);
        fstatus = OMGT_STATUS_SUCCESS;
    }
    else
        OMGT_DBGPRINT(port, "Got NULL response - FAILED\n");

    return (fstatus);
}

/* Deprecated. Use omgt_get_vf_port_stats2
*  This Should be removed in favor of the version that supports 32bit lids
*  when opamgt public library decides breaking compatibility is acceptable. 
*/

OMGT_STATUS_T
omgt_pa_get_vf_port_stats(
    struct omgt_port         *port,
    STL_PA_IMAGE_ID_DATA     pm_image_id_query,
	char    		         *vf_name,
    uint16_t                 lid,
    uint8                    port_num,
    STL_PA_IMAGE_ID_DATA    *pm_image_id_resp,
    STL_PA_VF_PORT_COUNTERS_DATA *vf_port_counters,
    uint32                  *flags,
    uint32                   delta,
	uint32                   user_cntrs
    )
{
	OMGT_DBGPRINT(port, "omgt_pa_get_vf_port_stats is deprecated. Please use omgt_pa_get_vf_port_stats2\n");
	return omgt_pa_get_vf_port_stats2(port, pm_image_id_query, vf_name, lid, port_num, pm_image_id_resp, vf_port_counters, flags, delta, user_cntrs);
}

/**
 *  Freeze specified image
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of image to free. 
 * @param pm_image_id_resp      Pointer to image ID of image frozen. 
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Freeze successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_freeze_image(
    struct omgt_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp 
    )
{
    OMGT_STATUS_T          fstatus = OMGT_STATUS_ERROR;
    STL_PA_IMAGE_ID_DATA   image_id = pm_image_id_query;
    STL_PA_IMAGE_ID_DATA   *response;

    // Validate the parameters and state
    if (!port)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    OMGT_DBGPRINT(port, "QueryImageNum = 0x%" PRIX64 " Offset = %d\n",
              pm_image_id_query.imageNumber, (int)pm_image_id_query.imageOffset );

    if ( ( response =
           iba_pa_single_mad_freeze_image_response_query(
              port, &image_id ) ) != NULL )
    {
        OMGT_DBGPRINT(port, "RespImageNum = 0x%" PRIX64 " Offset = %d\n",
                  response->imageNumber, (int)response->imageOffset );
        
        if (pm_image_id_resp)
            *pm_image_id_resp = *response;
        
        MemoryDeallocate(response);
        fstatus = OMGT_STATUS_SUCCESS;
    }
    else
        OMGT_DBGPRINT(port, "Got NULL response - FAILED\n");

    return (fstatus);
}

/**
 *   Move freeze of image 1 to image 2
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of frozen image 1. 
 * @param pm_image_id_resp      Pointer to image ID of image2. 
 *  
 * @return 
 *   OMGT_STATUS_SUCCESS       - Move image Freeze successful
 *   OMGT_STATUS_UNAVAILABLE   - Image 2 unavailable freeze
 *   OMGT_STATUS_ERROR         - Error
 */
OMGT_STATUS_T
omgt_pa_move_image_freeze(
    struct omgt_port     *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id1,
    STL_PA_IMAGE_ID_DATA       *pm_image_id2 
    )
{
    FSTATUS fstatus     = OMGT_STATUS_ERROR;
    STL_MOVE_FREEZE_DATA    request;
    STL_MOVE_FREEZE_DATA    *response;

    // Validate the parameters and state
    if (!port || !pm_image_id2)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    OMGT_DBGPRINT(port, "Img1ImageNum = 0x%" PRIX64 " Offset = %d\n",
              pm_image_id1.imageNumber, (int)pm_image_id1.imageOffset );
    OMGT_DBGPRINT(port, "Img2ImageNum = 0x%" PRIX64 " Offset = %d\n",
              pm_image_id2->imageNumber, (int)pm_image_id2->imageOffset );
    request.oldFreezeImage = pm_image_id1;
    request.newFreezeImage = *pm_image_id2;

    if ( ( response =
           iba_pa_single_mad_move_freeze_response_query( 
              port, &request ) ) != NULL )
    {
        OMGT_DBGPRINT(port, "RespOldImageNum = 0x%" PRIX64 " Offset = %d\n",
                  response->oldFreezeImage.imageNumber, response->oldFreezeImage.imageOffset );
        OMGT_DBGPRINT(port, "RespNewImageNum = 0x%" PRIX64 " Offset = %d\n",
                  response->newFreezeImage.imageNumber, response->newFreezeImage.imageOffset );
         
        *pm_image_id2 = response->newFreezeImage;
        fstatus = OMGT_STATUS_SUCCESS;
        MemoryDeallocate(response);
    }
    else
    {
        OMGT_DBGPRINT(port, "Got NULL response - UNAVAILABLE\n");
        fstatus = OMGT_STATUS_UNAVAILABLE;
    }

    return (fstatus);
}

/**
 *   Release specified image.
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of image to release. 
 *  
 * @return 
 *   OMGT_STATUS_SUCCESS       - Release successful
 *     OMGT_STATUS_ERROR       - Error
 */
OMGT_STATUS_T
omgt_pa_release_image(
    struct omgt_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query
    )
{
    OMGT_STATUS_T          fstatus = OMGT_STATUS_ERROR;
    STL_PA_IMAGE_ID_DATA   image_id = pm_image_id_query;
    STL_PA_IMAGE_ID_DATA   *response;

    // Validate the parameters and state
    if (!port)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    if ( ( response = iba_pa_single_mad_release_image_response_query( 
               port, &image_id ) ) != NULL )
    {
        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port, "QueryImageNum = 0x%" PRIX64 " Offset = %d\n",
                  pm_image_id_query.imageNumber, (int)pm_image_id_query.imageOffset );
        OMGT_DBGPRINT(port, "RespImageNum = 0x%" PRIX64 " Offset = %d\n",
                  response->imageNumber, (int)response->imageOffset );
        
        MemoryDeallocate(response);
        fstatus = OMGT_STATUS_SUCCESS;
    }
    else
        OMGT_DBGPRINT(port, "Got NULL response - FAILED\n");

    return (fstatus);
}

/**
 *   Renew lease of specified image.
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of image to renew. 
 *  
 * @return 
 *   OMGT_STATUS_SUCCESS       - Renew successful
 *     OMGT_STATUS_ERROR       - Error
 */
OMGT_STATUS_T
omgt_pa_renew_image(
    struct omgt_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query
    )
{
    OMGT_STATUS_T          fstatus = OMGT_STATUS_ERROR;
    STL_PA_IMAGE_ID_DATA   image_id = pm_image_id_query;
    STL_PA_IMAGE_ID_DATA   *response;

    // Validate the parameters and state
    if (!port)
    {
        OMGT_OUTPUT_ERROR(port, "invalid params or state\n");
        return (fstatus);
    }

    if ( ( response = iba_pa_single_mad_renew_image_response_query( 
       port, &image_id ) ) != NULL )
    {
        // TBD remove some of these after additional MAD testing
        OMGT_DBGPRINT(port, "QueryImageNum = 0x%" PRIX64 " Offset = %d\n",
                  pm_image_id_query.imageNumber, (int)pm_image_id_query.imageOffset );
        OMGT_DBGPRINT(port, "RespImageNum = 0x%" PRIX64 " Offset = %d\n",
                  response->imageNumber, (int)response->imageOffset );
        
        MemoryDeallocate(response);
        fstatus = OMGT_STATUS_SUCCESS;
    }
    else
        OMGT_DBGPRINT(port, "Got NULL response - FAILED\n");

    return (fstatus);
}

#endif
