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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <string.h>
#include <errno.h>

#define OPAMGT_PRIVATE 1

#include "iba/stl_sd.h"
#include "iba/stl_sa_priv.h"
#include "ib_mad.h"
#include "ib_sm_priv.h"
#include "ib_generalServices.h"
#include "ib_utils_openib.h"
#include "opamgt_sa_priv.h"
#include <infiniband/umad.h>


#define STL_TRACE_RECORD_COMP_ENCRYPT_MASK       0x55555555
#define PATHRECORD_NUMBPATH				32		// max to request given 2 guids

#define OMGT_NUM_MGMT_CLASSES 4

static const char* const SdQueryInputTypeText[] = {
	"InputTypeNoInput",
	"InputTypeNodeType",
	"InputTypeSystemImageGuid",
	"InputTypeNodeGuid",
	"InputTypePortGuid",
	"InputTypePortGid",
	"InputTypeMcGid",
	"InputTypePortGuidPair",
	"InputTypeGidPair",
	"InputTypePathRecord",
#ifdef IB_STACK_OPENIB
	"InputTypePathRecordNetworkOrder",
#endif
	"InputTypeLid",
	"InputTypePKey",
	"InputTypeSL",
	"InputTypeIndex",
	"InputTypeServiceId",
	"InputTypeNodeDesc",
	"InputTypeServiceRecord",
	"InputTypeMcMemberRecord",
	"InputTypePortGuidList",
	"InputTypeGidList",
	"InputTypeMultiPathRecord",
};

static const char* const SdQueryResultTypeText[] = {
	"OutputTypeSystemImageGuid",
	"OutputTypeNodeGuid",
	"OutputTypePortGuid",
	"OutputTypeLid",
	"OutputTypeGid",
	"OutputTypeNodeDesc",
	"OutputTypePathRecord",
#ifdef IB_STACK_OPENIB
	"OutputTypePathRecordNetworkOrder",
#endif
	"OutputTypeNodeRecord",
	"OutputTypePortInfoRecord",
	"OutputTypeSMInfoRecord",
	"OutputTypeLinkRecord",
	"OutputTypeServiceRecord",
	"OutputTypeMcMemberRecord",
	"OutputTypeInformInfoRecord",
	"OutputTypeTraceRecord",
	"OutputTypeSwitchInfoRecord",
	"OutputTypeLinearFDBRecord",
	"OutputTypeRandomFDBRecord",
	"OutputTypeMCastFDBRecord",
	"OutputTypeVLArbTableRecord",
	"OutputTypePKeyTableRecord",
	"OutputTypeVfInfoRecord",
	"OutputTypeClassPortInfo",
	// PM query results
	"OutputTypePaRecord",
	"OutputTypePaTableRecord",
};

// See IbAccess/Common/Inc/stl_sd.h for the OutputTypeStlBase defines
static const char* const SdQueryStlResultTypeText[] = {
	"OutputTypeStlUndefined",
	"OutputTypeStlNodeRecord",
	"OutputTypeStlNodeDesc",
	"OutputTypeStlPortInfoRecord",
	"OutputTypeStlSwitchInfoRecord",
	"OutputTypeStlPkeyTableRecord",
	"OutputTypeStlSLSCTableRecord",
	"OutputTypeStlSMInfoRecord",
	"OutputTypeStlLinearFDBRecord",
	"OutputTypeStlVLArbTableRecord",
	"OutputTypeStlMcMemberRecord",
	"OutputTypeStlLid",
	"OutputTypeStlMCastFDBRecord",
	"OutputTypeStlLinkRecord",
	"OutputTypeStlSystemImageGuid",
	"OutputTypeStlPortGuid",
	"OutputTypeStlNodeGuid",
	"OutputTypeStlServiceRecord",
	"OutputTypeStlInformInfoRecord",
	"OutputTypeStlVfInfo",
	"OutputTypeStlTraceRecord",
	"OutputTypeStlQuarantinedNodeRecord",
    "OutputTypeStlCongInfoRecord",
    "OutputTypeStlSwitchCongRecord",
    "OutputTypeStlSwitchPortCongRecord",
    "OutputTypeStlHFICongRecord",
    "OutputTypeStlHFICongCtrlRecord",
	"OutputTypeStlBufCtrlTabRecord",
	"OutputTypeStlCableInfoRecord",
	"OutputTypeStlPortGroupRecord",
	"OutputTypeStlPortGroupFwdRecord",
	"OutputTypeStlSCSLTableRecord",
	"OutputTypeStlSCVLtTableRecord",
	"OutputTypeStlSCVLntTableRecord",
	"OutputTypeStlSCSCTableRecord",
	"OutputTypeStlClassPortInfo",
	"OutputTypeStlFabricInfoRecord",
	"OutputTypeStlSwitchCostRecord"
};

/* TBD get from libibt? */
static const char * const MadInvalidFieldText[] = {
	"Valid Field",	// should not be used below
	"Unsupported Class or Version",
	"Unsupported Method",
	"Unsupported Method/Attribute Combination",
	"Reserved Invalid Field 4",
	"Reserved Invalid Field 5",
	"Reserved Invalid Field 6",
	"Invalid Attribute or Attribute Modifier"
};

/* upper 8 bits of MAD_STATUS_SA_* fields from ib_sa_records.h */
static const char* const SdSAStatusText[] = {
	"Success",	// not used by code below
	"Insufficient SA Resources",
	"No SA Records",
	"Invalid SA Request",
	"Too Many SA Records",
	"Invalid GID in SA Request",
	"Insufficient Components in SA Request"
};

const char* iba_sd_query_input_type_msg(QUERY_INPUT_TYPE code)
{
	if (code < 0 || code >= (int)(sizeof(SdQueryInputTypeText)/sizeof(char*)))
		return "Unknown SD Query Input Type";
	else
		return SdQueryInputTypeText[code];
}

const char* iba_sd_query_result_type_msg(QUERY_RESULT_TYPE code)
{
	if (code < 0 || code >= (int)(sizeof(SdQueryResultTypeText)/sizeof(char*))) {
		code -= OutputTypeStlBase;
		if (code < 0 || code >= (int)(sizeof(SdQueryStlResultTypeText)/sizeof(char*)))
			return "Unknown SD Query Result Type";
		else
			return SdQueryStlResultTypeText[code];
	} else
		return SdQueryResultTypeText[code];
}

const char* iba_mad_status_msg(MAD_STATUS madStatus)
{
	if (madStatus.AsReg16 == MAD_STATUS_SUCCESS)
		return "Success";
	else if (madStatus.S.Busy)
		return "Busy";
	else if (madStatus.S.RedirectRqrd)
		return "Redirection Required";
	else if (madStatus.S.InvalidField)
		return MadInvalidFieldText[madStatus.S.InvalidField];
	else if (madStatus.S.ClassSpecific)
		return "Class Specific Status";
	else
		return "Unknown Mad Status";
}

// this function is useful when we have DR.s.Status which is a 15 bit bitfield
const char* iba_mad_status_msg2(uint16 madStatus)
{
	MAD_STATUS mad_status;

	mad_status.AsReg16 = madStatus;
	return iba_mad_status_msg(mad_status);
}

const char* iba_sd_mad_status_msg(uint32 code)
{
	// this is a little more complex than most due to bitfields and reserved
	// values
	MAD_STATUS madStatus;

	madStatus.AsReg16 = code; // ignore reserved bits in Sd MadStatus
	if (code == MAD_STATUS_SUCCESS || (code & 0xff))
		return iba_mad_status_msg(madStatus);	// standard mad status fields
	else {
		code = madStatus.S.ClassSpecific;	// SA specific status code field
		if (code >= (unsigned)(sizeof(SdSAStatusText)/sizeof(char*)))
			return "Unknown SA Mad Status";
		else
			return SdSAStatusText[code];
	}
}

/**
 * Common actions related to the query of the SA
 *  
 * @param pSA            pointer to the SA query message
 * @param ppRsp          pointer to the address of the SA query 
 *  					 response
 * @param record_size    Size of the record being queried.
 * @param ppQR           pointer to the address of the response 
 *  					 values
 * @param port           port opened by omgt_open_port_* 
 *  
 * @return          0 if success, else error code (FSTATUS)
 */
static FSTATUS sa_query_common(SA_MAD * pSA, SA_MAD **ppRsp, uint32_t record_size,
	PQUERY_RESULT_VALUES *ppQR, struct omgt_port *port)
{
	FSTATUS    fstatus = FSUCCESS;
	MAD_STATUS madStatus = {0};
	size_t     length = 0;
	uint32_t   cnt = 0;
	uint32_t   memsize;
	struct omgt_mad_addr addr = {0};

	DBG_ENTER_FUNC(port);

	/* If port is In-Band, set up addr */
	if (!port->is_oob_enabled) {
		uint8_t port_state;
		(void)omgt_port_get_port_state(port, &port_state);
		if (port_state != PortStateActive) {
			OMGT_OUTPUT_ERROR(port, "Local port not Active!\n");
			return FINVALID_STATE;
		}
		(void)omgt_port_get_port_sm_lid(port, &addr.lid);
		(void)omgt_port_get_port_sm_sl(port, &addr.sl);
		addr.qpn = 1;
		addr.qkey = QP1_WELL_KNOWN_Q_KEY;
		addr.pkey = OMGT_DEFAULT_PKEY;
	}

	// Default the parameters to failed state.
	*ppQR = NULL;
	*ppRsp= NULL;

	// If the port is In-Band, use the correct PKEY.
	// Should attempt to use full mgmt if available,
	// which is default case above.
	if (!port->is_oob_enabled && omgt_find_pkey(port, 0xffff) < 0) {
		// No full mgmt pkey available on this node.
		switch (pSA->common.AttributeID) {
        case SA_ATTRIB_PORTINFO_RECORD:
            if (pSA->common.BaseVersion != IB_BASE_VERSION) {
                fstatus = FPROTECTION;
                goto done;
            }
			// fall through
		case SA_ATTRIB_PATH_RECORD:
		case SA_ATTRIB_MULTIPATH_RECORD:
		case SA_ATTRIB_NODE_RECORD:
		case SA_ATTRIB_INFORM_INFO:
		case SA_ATTRIB_INFORM_INFO_RECORD:
		case SA_ATTRIB_NOTICE:
		case SA_ATTRIB_SERVICE_RECORD:
		case SA_ATTRIB_MCMEMBER_RECORD:
		case SA_ATTRIB_CLASS_PORT_INFO:
			// These attributes can use limited mgmt pkey, if available.
			if (omgt_find_pkey(port, 0x7fff) >= 0) {
				addr.pkey = 0x7fff;
				break;
			}
			// fall through
		default:
			// Unable to make such a query.
			// Must be full or limited mgmt, and we do not have proper pkey in our tables.
			fstatus = FPROTECTION;
			goto done;
		}
	}

	/* Set a short timeout for class port info as these queries will always be
	 * a small constant size.
	 */
	int timeout;
	if (pSA->common.AttributeID == SA_ATTRIB_CLASS_PORT_INFO && port->sa_service_state != OMGT_SERVICE_STATE_OPERATIONAL) {
		timeout = 250; //250 ms
	} else {
		timeout = port->ms_timeout;
	}

	BSWAP_SA_HDR (&pSA->SaHdr);
	BSWAP_MAD_HEADER((MAD*)pSA);

	fstatus = omgt_send_recv_mad_alloc(port, (uint8_t *)pSA, record_size + sizeof(MAD_COMMON) + sizeof(SA_MAD_HDR),
		&addr, (uint8_t **)ppRsp, &length, timeout, port->retry_count);

	if (fstatus != FSUCCESS) {
		OMGT_DBGPRINT(port, "Query SA failed to send: %d\n", fstatus);
		goto done;
	}
	/* Check if MAD header is present */
	if (length < sizeof(MAD_COMMON)) {
		OMGT_DBGPRINT(port, "Query SA: Failed to receive packet: length (%zu) less than sizeof(MAD_COMMON) (%zu)\n",
			length, sizeof(MAD_COMMON));
		fstatus = FNOT_FOUND;
		goto done;
	}
	BSWAP_MAD_HEADER((MAD*)(*ppRsp));
	MAD_GET_STATUS((*ppRsp), &madStatus);
	port->sa_mad_status = madStatus.AsReg16;

	/* Check if SA Header is present */
	if (length < IBA_SUBN_ADM_HDRSIZE) {
		OMGT_DBGPRINT(port, "Query SA: Failed to receive packet: length (%zu) less than IBA_SUBN_ADM_HDRSIZE (%u)\n",
			length, IBA_SUBN_ADM_HDRSIZE);
		fstatus = FNOT_FOUND;
		goto done;
	}
	BSWAP_SA_HDR (&(*ppRsp)->SaHdr);

	// dump of SA header for debug
	OMGT_DBGPRINT(port, " SA Header\n");
	OMGT_DBGPRINT(port, " length %zu (0x%zx) vs IBA_SUBN_ADM_HDRSIZE %d\n",
		length, length, IBA_SUBN_ADM_HDRSIZE);
	OMGT_DBGPRINT(port, " SmKey (0x%016"PRIx64")\n", (*ppRsp)->SaHdr.SmKey);
	OMGT_DBGPRINT(port, " AttributeOffset %u (0x%x) : in bytes: %u\n",
		(*ppRsp)->SaHdr.AttributeOffset, (*ppRsp)->SaHdr.AttributeOffset,
		((*ppRsp)->SaHdr.AttributeOffset * 8));
	OMGT_DBGPRINT(port, " Reserved (0x%x)\n", (*ppRsp)->SaHdr.Reserved);
	OMGT_DBGPRINT(port, " ComponetMask (0x%016"PRIx64")\n", (*ppRsp)->SaHdr.ComponentMask);

	// if no records IBTA 1.2.1 says AttributeOffset should be 0
	if ((*ppRsp)->common.mr.AsReg8 == SUBN_ADM_GET_RESP) {
		cnt = 1; /* Count is always 1 for a GET(). */
	} else if ((*ppRsp)->SaHdr.AttributeOffset) {
		/* Count is data length / attribute offset. */
		cnt = (int)((length - IBA_SUBN_ADM_HDRSIZE) /
					((*ppRsp)->SaHdr.AttributeOffset * sizeof(uint64)));
	} else {
		cnt = 0;
	}

	OMGT_DBGPRINT(port, "Record count is %d\n", cnt);

	// If the command was not successful (mad status is not 0)
	// Change overall return status to failed.
	// Fill in the result pointer so the caller can evaluate the return mad status.
	if (madStatus.AsReg16!=0) {
		OMGT_DBGPRINT(port, "Query SA failed: Mad status is 0x%x: %s\n",
			madStatus.AsReg16, iba_sd_mad_status_msg(madStatus.AsReg16));
	}

	// Query result is the size of one of the IBACCESS expected data types.
	// Multiply that size by the count of records we got back.
	// Add to that size, the size of the size of the query response struct.
	memsize  = record_size * cnt;
	memsize += sizeof (uint32_t); 
	memsize += sizeof (QUERY_RESULT_VALUES);

	// ResultDataSize should be 0 when status is not successful and no data is returned
	*ppQR = MemoryAllocate2AndClear(memsize, IBA_MEM_FLAG_PREMPTABLE, OMGT_MEMORY_TAG);
	if (!(*ppQR)) {
		fstatus = FINSUFFICIENT_MEMORY;
		OMGT_DBGPRINT(port, "Query SA failed to allocate result: %d\n", fstatus);
		goto done;
	}

	(*ppQR)->Status = fstatus;
	(*ppQR)->MadStatus = madStatus.AsReg16;
	(*ppQR)->ResultDataSize = record_size * cnt;
	*((uint32_t*)((*ppQR)->QueryResult)) = cnt;

done:
	if (fstatus != FSUCCESS) {
		if (*ppRsp!=NULL) {
			free (*ppRsp);
			*ppRsp = NULL;
		}
	}

	DBG_EXIT_FUNC(port);
	return (fstatus);
}

/**
 * Fill in the query node info
 *  
 * @param pSA           pointer to the MAD header
 * @param pQuery        pointer to the query message
 *  
 * @return          0 if success, else error code (FSTATUS)
 */
static FSTATUS fillInNodeRecord(SA_MAD * pSA, POMGT_QUERY pQuery)
{
	STL_NODE_RECORD *pNR = NULL;

	if ((pSA==NULL) || (pQuery==NULL)) {
		return (FERROR);
	}

	MAD_SET_ATTRIB_ID(pSA, STL_SA_ATTR_NODE_RECORD);
	pNR = (STL_NODE_RECORD *)pSA->Data;
	pNR->Reserved = 0;

	switch (pQuery->InputType) {
	case InputTypeNoInput:     
		pSA->SaHdr.ComponentMask = 0;
		break;
	case InputTypeLid:           
		pSA->SaHdr.ComponentMask = STL_NODE_RECORD_COMP_LID;
		pNR->RID.LID = pQuery->InputValue.NodeRecord.Lid;
		break;
	case InputTypePortGuid:
		pSA->SaHdr.ComponentMask = STL_NODE_RECORD_COMP_PORTGUID;
		pNR->NodeInfo.PortGUID = pQuery->InputValue.NodeRecord.PortGUID;
		break;
	case InputTypeNodeType:
		pSA->SaHdr.ComponentMask = STL_NODE_RECORD_COMP_NODETYPE;
		pNR->NodeInfo.NodeType = pQuery->InputValue.NodeRecord.NodeType;
		break;
	case InputTypeSystemImageGuid:
		pSA->SaHdr.ComponentMask = STL_NODE_RECORD_COMP_SYSIMAGEGUID;
		pNR->NodeInfo.SystemImageGUID = pQuery->InputValue.NodeRecord.SystemImageGUID;
		break;
	case InputTypeNodeGuid:
		pSA->SaHdr.ComponentMask = STL_NODE_RECORD_COMP_NODEGUID;
		pNR->NodeInfo.NodeGUID = pQuery->InputValue.NodeRecord.SystemImageGUID;
		break;
	case InputTypeNodeDesc:
		pSA->SaHdr.ComponentMask = STL_NODE_RECORD_COMP_NODEDESC;
		memcpy(pNR->NodeDesc.NodeString, pQuery->InputValue.NodeRecord.NodeDesc,
			STL_NODE_DESCRIPTION_ARRAY_SIZE);
		break;
	default:
		fprintf(stderr, "Query not supported in OFED: Input=%s(%d), Output=%s(%d)\n",
				iba_sd_query_input_type_msg(pQuery->InputType),
				pQuery->InputType,
				iba_sd_query_result_type_msg(pQuery->OutputType),
				pQuery->OutputType);
		return(FINVALID_PARAMETER);
	}

	BSWAP_STL_NODE_RECORD(pNR);

	return(FSUCCESS);
}

/**
 * Fill in node IB info
 *  
 * @param pSA           pointer to the mad header
 * @param pQuery        pointer to the query message
 *  
 * @return          0 if success, else error code (FSTATUS)
 */
static FSTATUS fillInIbNodeRecord(SA_MAD * pSA, POMGT_QUERY pQuery)
{
	IB_NODE_RECORD *pNR = NULL;

	if ((pSA==NULL) || (pQuery==NULL)) {
		return (FERROR);
	}

	MAD_SET_ATTRIB_ID(pSA, SA_ATTRIB_NODE_RECORD);
	pNR = (IB_NODE_RECORD *)pSA->Data;

	switch (pQuery->InputType) {
	case InputTypeNoInput:     
		pSA->SaHdr.ComponentMask = 0;
		break;
	case InputTypeLid:           
		pSA->SaHdr.ComponentMask = IB_NODE_RECORD_COMP_LID;
		pNR->RID.s.LID = pQuery->InputValue.IbNodeRecord.Lid;
		break;
	case InputTypePortGuid:
		pSA->SaHdr.ComponentMask = IB_NODE_RECORD_COMP_PORTGUID;
        pNR->NodeInfoData.PortGUID = pQuery->InputValue.IbNodeRecord.PortGUID;
		break;
	case InputTypeNodeType:
		pSA->SaHdr.ComponentMask = IB_NODE_RECORD_COMP_NODETYPE;
		pNR->NodeInfoData.NodeType = pQuery->InputValue.IbNodeRecord.NodeType;
		break;
	case InputTypeSystemImageGuid:
		pSA->SaHdr.ComponentMask = IB_NODE_RECORD_COMP_SYSIMAGEGUID;
		pNR->NodeInfoData.SystemImageGUID = pQuery->InputValue.IbNodeRecord.SystemImageGUID;
		break;
	case InputTypeNodeGuid:
		pSA->SaHdr.ComponentMask = IB_NODE_RECORD_COMP_NODEGUID;
		pNR->NodeInfoData.NodeGUID = pQuery->InputValue.IbNodeRecord.NodeGUID;
		break;
	case InputTypeNodeDesc:
		pSA->SaHdr.ComponentMask = IB_NODE_RECORD_COMP_NODEDESC;
		memcpy(pNR->NodeDescData.NodeString, pQuery->InputValue.IbNodeRecord.NodeDesc,
			STL_NODE_DESCRIPTION_ARRAY_SIZE);
		break;
	default:
		fprintf(stderr, "Query not supported in OFED: Input=%s(%d), Output=%s(%d)\n",
				iba_sd_query_input_type_msg(pQuery->InputType),
				pQuery->InputType,
				iba_sd_query_result_type_msg(pQuery->OutputType),
				pQuery->OutputType);
		return(FINVALID_PARAMETER);
	}

	BSWAP_IB_NODE_RECORD(pNR);

	return(FSUCCESS);
}

/**
 * Free the memory used in the query result
 *  
 * @param pQueryResult    pointer to the SA query result buffer
 *  
 * @return          none
 */
void omgt_free_query_result_buffer(IN void * pQueryResult)
{
	MemoryDeallocate(pQueryResult);
}


FSTATUS omgt_input_value_conversion(OMGT_QUERY *output_query, QUERY_INPUT_VALUE *old_query, IB_GID source_gid)
{
	switch ((int)output_query->OutputType) {
		/* InputTypeNoInput Only */
	case OutputTypeClassPortInfo:
	case OutputTypeStlClassPortInfo:
	case OutputTypeStlSMInfoRecord:
	case OutputTypeStlFabricInfoRecord:
	case OutputTypeStlQuarantinedNodeRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeNodeRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.IbNodeRecord.Lid = old_query->Lid;
			break;
		case InputTypePortGuid:
			output_query->InputValue.IbNodeRecord.PortGUID = old_query->Guid;
			break;
		case InputTypeNodeGuid:
			output_query->InputValue.IbNodeRecord.NodeGUID = old_query->Guid;
			break;
		case InputTypeSystemImageGuid:
			output_query->InputValue.IbNodeRecord.SystemImageGUID = old_query->Guid;
			break;
		case InputTypeNodeType:
			output_query->InputValue.IbNodeRecord.NodeType = old_query->TypeOfNode;
			break;
		case InputTypeNodeDesc:
			memcpy(output_query->InputValue.IbNodeRecord.NodeDesc,
				old_query->NodeDesc.Name, STL_NODE_DESCRIPTION_ARRAY_SIZE);
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlNodeRecord:
	case OutputTypeStlNodeDesc:
	case OutputTypeStlLid:
	case OutputTypeStlSystemImageGuid:
	case OutputTypeStlPortGuid:
	case OutputTypeStlNodeGuid:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.NodeRecord.Lid = old_query->Lid;
			break;
		case InputTypePortGuid:
			output_query->InputValue.NodeRecord.PortGUID = old_query->Guid;
			break;
		case InputTypeNodeGuid:
			output_query->InputValue.NodeRecord.NodeGUID = old_query->Guid;
			break;
		case InputTypeSystemImageGuid:
			output_query->InputValue.NodeRecord.SystemImageGUID = old_query->Guid;
			break;
		case InputTypeNodeType:
			output_query->InputValue.NodeRecord.NodeType = old_query->TypeOfNode;
			break;
		case InputTypeNodeDesc:
			memcpy(output_query->InputValue.NodeRecord.NodeDesc,
				old_query->NodeDesc.Name, STL_NODE_DESCRIPTION_ARRAY_SIZE);
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypePortInfoRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.IbPortInfoRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlPortInfoRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.PortInfoRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlLinkRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.LinkRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlSwitchInfoRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.SwitchInfoRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypePathRecord:
	case OutputTypePathRecordNetworkOrder:
		switch (output_query->InputType) {
		case InputTypePortGuidList:
			output_query->InputValue.IbPathRecord.PortGuidList.SourceGuidCount =
				old_query->PortGuidList.SourceGuidCount;
			output_query->InputValue.IbPathRecord.PortGuidList.DestGuidCount =
				old_query->PortGuidList.DestGuidCount;
			memcpy(output_query->InputValue.IbPathRecord.PortGuidList.GuidList,
				old_query->PortGuidList.GuidList, MULTIPATH_GID_LIMIT * sizeof(EUI64));
			/* SharedSubnetPrefix is new field for this InputType */
			if (source_gid.Type.Global.SubnetPrefix == 0) return FERROR;
			output_query->InputValue.IbPathRecord.PortGuidList.SharedSubnetPrefix =
				source_gid.Type.Global.SubnetPrefix;
			break;
		case InputTypeGidList:
			output_query->InputValue.IbPathRecord.GidList.SourceGidCount =
				old_query->GidList.SourceGidCount;
			output_query->InputValue.IbPathRecord.GidList.DestGidCount =
				old_query->GidList.DestGidCount;
			memcpy(output_query->InputValue.IbPathRecord.GidList.GidList,
				old_query->GidList.GidList, MULTIPATH_GID_LIMIT * sizeof(IB_GID));
			break;
		case InputTypeMultiPathRecord:
			output_query->InputValue.IbPathRecord.MultiPathRecord.ComponentMask =
				old_query->MultiPathRecordValue.ComponentMask;
			output_query->InputValue.IbPathRecord.MultiPathRecord.MultiPathRecord =
				old_query->MultiPathRecordValue.MultiPathRecord;
			memcpy(output_query->InputValue.IbPathRecord.MultiPathRecord.Gids,
				old_query->MultiPathRecordValue.Gids, (MULTIPATH_GID_LIMIT - 1) * sizeof(IB_GID));
			break;
		case InputTypeSourceGid:
		case InputTypeNoInput:
			/* SourceGid is a new field for this InputType */
			if (source_gid.AsReg64s.H == 0 || source_gid.AsReg64s.L == 0) return FERROR;
			output_query->InputValue.IbPathRecord.SourceGid = source_gid;
			break;
		case InputTypePKey:
			output_query->InputValue.IbPathRecord.PKey.PKey = old_query->PKey;
			/* SourceGid is a new field for this InputType */
			if (source_gid.AsReg64s.H == 0 || source_gid.AsReg64s.L == 0) return FERROR;
			output_query->InputValue.IbPathRecord.PKey.SourceGid = source_gid;
			break;
		case InputTypeSL:
			output_query->InputValue.IbPathRecord.SL.SL = old_query->SL;
			/* SourceGid is a new field for this InputType */
			if (source_gid.AsReg64s.H == 0 || source_gid.AsReg64s.L == 0) return FERROR;
			output_query->InputValue.IbPathRecord.SL.SourceGid = source_gid;
			break;
		case InputTypeServiceId:
			output_query->InputValue.IbPathRecord.ServiceId.ServiceId = old_query->ServiceId;
			/* SourceGid is a new field for this InputType */
			if (source_gid.AsReg64s.H == 0 || source_gid.AsReg64s.L == 0) return FERROR;
			output_query->InputValue.IbPathRecord.ServiceId.SourceGid = source_gid;
			break;
		case InputTypePortGuid:
			output_query->InputValue.IbPathRecord.PortGuid.DestPortGuid =
				old_query->Guid;
			/* SourcePortGuid is a new field for this InputType */
			if (source_gid.Type.Global.InterfaceID == 0) return FERROR;
			output_query->InputValue.IbPathRecord.PortGuid.SourcePortGuid =
				source_gid.Type.Global.InterfaceID;
			/* SharedSubnetPrefix is new field for this InputType */
			if (source_gid.Type.Global.SubnetPrefix == 0) return FERROR;
			output_query->InputValue.IbPathRecord.PortGuid.SharedSubnetPrefix =
				source_gid.Type.Global.SubnetPrefix;
			break;
		case InputTypePortGuidPair:
			output_query->InputValue.IbPathRecord.PortGuidPair.SourcePortGuid =
				old_query->PortGuidPair.SourcePortGuid;
			output_query->InputValue.IbPathRecord.PortGuidPair.DestPortGuid =
				old_query->PortGuidPair.DestPortGuid;
			/* SharedSubnetPrefix is new field for this InputType */
			if (source_gid.Type.Global.SubnetPrefix == 0) return FERROR;
			output_query->InputValue.IbPathRecord.PortGuidPair.SharedSubnetPrefix =
				source_gid.Type.Global.SubnetPrefix;
			break;
		case InputTypePortGid:
			output_query->InputValue.IbPathRecord.PortGid.DestGid =
				old_query->Gid;
			/* SourceGid is a new field for this InputType */
			if (source_gid.AsReg64s.H == 0 || source_gid.AsReg64s.L == 0) return FERROR;
			output_query->InputValue.IbPathRecord.PortGid.SourceGid = source_gid;
			break;
		case InputTypeGidPair:
			output_query->InputValue.IbPathRecord.GidPair.DestGid =
				old_query->GidPair.DestGid;
			output_query->InputValue.IbPathRecord.GidPair.SourceGid =
				old_query->GidPair.SourceGid;
			break;
		case InputTypeLid:
			output_query->InputValue.IbPathRecord.Lid.DLid =
				old_query->Lid;
			/* SourceGid is a new field for this InputType */
			if (source_gid.AsReg64s.H == 0 || source_gid.AsReg64s.L == 0) return FERROR;
			output_query->InputValue.IbPathRecord.Lid.SourceGid = source_gid;
			break;
		case InputTypePathRecord:
		case InputTypePathRecordNetworkOrder:
			output_query->InputValue.IbPathRecord.PathRecord.ComponentMask =
				old_query->PathRecordValue.ComponentMask;
			output_query->InputValue.IbPathRecord.PathRecord.PathRecord =
				old_query->PathRecordValue.PathRecord;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlTraceRecord:
		switch (output_query->InputType) {
		case InputTypePathRecord:
			output_query->InputValue.TraceRecord.PathRecord.ComponentMask =
				old_query->PathRecordValue.ComponentMask;
			output_query->InputValue.TraceRecord.PathRecord.PathRecord =
				old_query->PathRecordValue.PathRecord;
			break;
		case InputTypePortGuid:
			output_query->InputValue.TraceRecord.PortGuid.DestPortGuid =
				old_query->Guid;
			/* SourcePortGuid is a new field for this InputType */
			if (source_gid.Type.Global.InterfaceID == 0) return FERROR;
			output_query->InputValue.TraceRecord.PortGuid.SourcePortGuid =
				source_gid.Type.Global.InterfaceID;
			/* SharedSubnetPrefix is new field for this InputType */
			if (source_gid.Type.Global.SubnetPrefix == 0) return FERROR;
			output_query->InputValue.TraceRecord.PortGuid.SharedSubnetPrefix =
				source_gid.Type.Global.SubnetPrefix;
			break;
		case InputTypePortGuidPair:
			output_query->InputValue.TraceRecord.PortGuidPair.SourcePortGuid =
				old_query->PortGuidPair.SourcePortGuid;
			output_query->InputValue.TraceRecord.PortGuidPair.DestPortGuid =
				old_query->PortGuidPair.DestPortGuid;
			/* SharedSubnetPrefix is new field for this InputType */
			if (source_gid.Type.Global.SubnetPrefix == 0) return FERROR;
			output_query->InputValue.TraceRecord.PortGuidPair.SharedSubnetPrefix =
				source_gid.Type.Global.SubnetPrefix;
			break;
		case InputTypePortGid:
			output_query->InputValue.TraceRecord.PortGid.DestGid =
				old_query->Gid;
			/* SourceGid is new field for this InputType */
			if (source_gid.AsReg64s.H == 0 || source_gid.AsReg64s.L == 0) return FERROR;
			output_query->InputValue.IbPathRecord.PortGid.SourceGid = source_gid;
			break;
		case InputTypeGidPair:
			output_query->InputValue.TraceRecord.GidPair.DestGid =
				old_query->GidPair.DestGid;
			output_query->InputValue.TraceRecord.GidPair.SourceGid =
				old_query->GidPair.SourceGid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeServiceRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypePortGid:
			output_query->InputValue.IbServiceRecord.ServiceGid = old_query->Gid;
			break;
		default: return FNOT_FOUND;
		}
		break;
#ifndef NO_STL_MCMEMBER_OUTPUT
	case OutputTypeStlMcMemberRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypePortGid:
			output_query->InputValue.StlMcMemberRecord.PortGid = old_query->Gid;
			break;
		case InputTypeMcGid:
			output_query->InputValue.StlMcMemberRecord.McGid = old_query->Gid;
			break;
		case InputTypeLid:
			output_query->InputValue.StlMcMemberRecord.Lid = old_query->Lid;
			break;
		case InputTypePKey:
			output_query->InputValue.StlMcMemberRecord.PKey = old_query->PKey;
			break;
		case InputTypeSL:
                        output_query->InputValue.StlMcMemberRecord.SL = old_query->SL;
                        break;
		default: return FNOT_FOUND;
		}
		break;
#endif
	case OutputTypeMcMemberRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypePortGid:
			output_query->InputValue.IbMcMemberRecord.PortGid = old_query->Gid;
			break;
		case InputTypeMcGid:
			output_query->InputValue.IbMcMemberRecord.McGid = old_query->Gid;
			break;
		case InputTypeLid:
			output_query->InputValue.IbMcMemberRecord.Lid = old_query->Lid;
			break;
		case InputTypePKey:
			output_query->InputValue.IbMcMemberRecord.PKey = old_query->PKey;
			break;
		case InputTypeSL:
                        output_query->InputValue.IbMcMemberRecord.SL = old_query->SL;
                        break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeInformInfoRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypePortGid:
			output_query->InputValue.IbInformInfoRecord.SubscriberGID = old_query->Gid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlInformInfoRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.StlInformInfoRecord.SubscriberLID = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlSCSCTableRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.ScScTableRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlSLSCTableRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.SlScTableRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlSCSLTableRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.ScSlTableRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlSCVLtTableRecord:
	case OutputTypeStlSCVLntTableRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.ScVlxTableRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlVLArbTableRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.VlArbTableRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlPKeyTableRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.PKeyTableRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlLinearFDBRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.LinFdbTableRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlMCastFDBRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.McFdbTableRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlVfInfoRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypePKey:
			output_query->InputValue.VfInfoRecord.PKey = old_query->PKey;
			break;
		case InputTypeSL:
			output_query->InputValue.VfInfoRecord.SL = old_query->SL;
			break;
		case InputTypeServiceId:
			output_query->InputValue.VfInfoRecord.ServiceId = old_query->ServiceId;
			break;
		case InputTypeMcGid:
			output_query->InputValue.VfInfoRecord.McGid = old_query->Gid;
			break;
		case InputTypeIndex:
			output_query->InputValue.VfInfoRecord.vfIndex = old_query->vfIndex;
			break;
		case InputTypeNodeDesc:
			snprintf(output_query->InputValue.VfInfoRecord.vfName,
				STL_VFABRIC_NAME_LEN, "%s", old_query->NodeDesc.Name);
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlCongInfoRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.CongInfoRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlSwitchCongRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.SwCongRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlSwitchPortCongRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.SwPortCongRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlHFICongRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.HFICongRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlHFICongCtrlRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.HFICongCtrlRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlBufCtrlTabRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.BufCtrlTableRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlCableInfoRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.CableInfoRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlPortGroupRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.PortGroupRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlPortGroupFwdRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.PortGroupFwdRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	case OutputTypeStlSwitchCostRecord:
		switch (output_query->InputType) {
		case InputTypeNoInput: break;
		case InputTypeLid:
			output_query->InputValue.SwitchCostRecord.Lid = old_query->Lid;
			break;
		default: return FNOT_FOUND;
		}
		break;
	default: return FNOT_FOUND;
	}
	return FSUCCESS;
}



/**
 * Pose a query to the fabric, expect a response.
 *  
 * @param port           port opened by omgt_open_port_* 
 * @param pQuery         pointer to the query structure
 * @param ppQueryResult  pointer where the response will go 
 *  
 * @return          0 if success, else error code
 */
static FSTATUS omgt_query_sa_internal(struct omgt_port *port, OMGT_QUERY *pQuery,
	struct _QUERY_RESULT_VALUES **ppQueryResult)
{
	FSTATUS fstatus = FSUCCESS;
	QUERY_RESULT_VALUES  *pQR = NULL;
	SA_MAD                mad;
	SA_MAD               *pRsp = NULL;
	static uint32_t       trans_id = 1;
    int i;

	DBG_ENTER_FUNC(port);

	/* All fields are supposed to be zeroed out if they are not used. */
	memset(&mad,0,sizeof(mad));

	// Setup defaults
	memset( &mad, 0, sizeof(mad));
	MAD_SET_METHOD_TYPE (&mad, SUBN_ADM_GETTABLE);
	MAD_SET_VERSION_INFO(&mad, STL_BASE_VERSION, MCLASS_SUBN_ADM, STL_SA_CLASS_VERSION);
	MAD_SET_TRANSACTION_ID(&mad, trans_id++); //<<16???

	// Process the command.
	switch ((int)pQuery->OutputType) {
	case OutputTypeClassPortInfo:
		{
			IB_CLASS_PORT_INFO_RESULTS *pCPIR;
			IB_CLASS_PORT_INFO *pCPI;

			if (pQuery->InputType != InputTypeNoInput)
				break;

			MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM, 
								 IB_SUBN_ADM_CLASS_VERSION);
			MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_CLASS_PORT_INFO);
			MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_GET);
			
			fstatus = sa_query_common(&mad, &pRsp, sizeof(IB_CLASS_PORT_INFO), &pQR, port);
			if (fstatus != FSUCCESS) break;

			pCPIR = (IB_CLASS_PORT_INFO_RESULTS*)pQR->QueryResult;
			pCPI = &pCPIR->ClassPortInfo[0];
			
			// There should only be one ClassPortInfo result.
			if (pCPIR->NumClassPortInfo > 0) {
				*pCPI = *((IB_CLASS_PORT_INFO*)(GET_RESULT_OFFSET(pRsp, 0)));
				BSWAP_IB_CLASS_PORT_INFO(pCPI);
			}
		}
		break;
	case OutputTypeStlClassPortInfo:
		{
			STL_CLASS_PORT_INFO_RESULT *pCPIR;
			STL_CLASS_PORT_INFO *pCPI;

			if (pQuery->InputType != InputTypeNoInput)
				break;

			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_CLASS_PORT_INFO);
			MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_GET);
			
			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_CLASS_PORT_INFO), &pQR, port);
			if (fstatus != FSUCCESS) break;

			pCPIR = (STL_CLASS_PORT_INFO_RESULT*)pQR->QueryResult;
			pCPI = &pCPIR->ClassPortInfo;
			
			// There should only be one ClassPortInfo result.
			if (pCPIR->NumClassPortInfo > 0) {
				*pCPI = *((STL_CLASS_PORT_INFO*)(GET_RESULT_OFFSET(pRsp, 0)));
				BSWAP_STL_CLASS_PORT_INFO(pCPI);
			}
		}
		break;
	case OutputTypeNodeRecord:       // Legacy, IB query
		{
			NODE_RECORD_RESULTS *pNRR;
			IB_NODE_RECORD      *pNR;

			MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM, IB_SUBN_ADM_CLASS_VERSION);

			// Take care of input types in fillIn...
			fstatus = fillInIbNodeRecord(&mad, pQuery);
			if (fstatus != FSUCCESS) break;

			fstatus = sa_query_common(&mad, &pRsp, sizeof (IB_NODE_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pNRR = (NODE_RECORD_RESULTS*)pQR->QueryResult;
			pNR = pNRR->NodeRecords;
			for (i=0; i< pNRR->NumNodeRecords; i++, pNR++) {
				*pNR =  * ((IB_NODE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_IB_NODE_RECORD(pNR);
			}
		}
		break;

	/* Group All STL NodeInfo Record based OutputTypes */
	case OutputTypeStlNodeRecord:
	case OutputTypeStlSystemImageGuid:
	case OutputTypeStlNodeGuid:
	case OutputTypeStlPortGuid:
	case OutputTypeStlLid:
	case OutputTypeStlNodeDesc:
		{
			STL_NODE_RECORD *pNR;

			fstatus = fillInNodeRecord(&mad, pQuery);
			if (fstatus != FSUCCESS) break;

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_NODE_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			switch ((int)pQuery->OutputType) {
			case OutputTypeStlNodeRecord:
				{
					STL_NODE_RECORD_RESULTS *pNRR = (STL_NODE_RECORD_RESULTS *)pQR->QueryResult;
					pNR = pNRR->NodeRecords;
					for (i=0; i< pNRR->NumNodeRecords; i++, pNR++) {
						*pNR = *((STL_NODE_RECORD *)(GET_RESULT_OFFSET(pRsp, i)));
						BSWAP_STL_NODE_RECORD(pNR);
					}
				}
				break;
			case OutputTypeStlSystemImageGuid:
				{
					GUID_RESULTS *pGR = (GUID_RESULTS*)pQR->QueryResult;
					EUI64 *pGuid = pGR->Guids;
					for (i=0; i< pGR->NumGuids; i++, pGuid++) {
						pNR = ((STL_NODE_RECORD *)(GET_RESULT_OFFSET(pRsp, i)));
						*pGuid = ntoh64(pNR->NodeInfo.SystemImageGUID);
					}
				}
				break;
			case OutputTypeStlNodeGuid:
				{
					GUID_RESULTS *pGR = (GUID_RESULTS*)pQR->QueryResult;
					EUI64 *pGuid = pGR->Guids;
					for (i=0; i< pGR->NumGuids; i++, pGuid++) {
						pNR = ((STL_NODE_RECORD *)(GET_RESULT_OFFSET(pRsp, i)));
						*pGuid = ntoh64(pNR->NodeInfo.NodeGUID);
					}
				}
				break;
			case OutputTypeStlPortGuid:
				{
					GUID_RESULTS *pGR = (GUID_RESULTS*)pQR->QueryResult;
					EUI64 *pGuid = pGR->Guids;
					for (i=0; i< pGR->NumGuids; i++, pGuid++) {
						pNR = ((STL_NODE_RECORD *)(GET_RESULT_OFFSET(pRsp, i)));
						*pGuid = ntoh64(pNR->NodeInfo.PortGUID);
					}
				}
				break;
			case OutputTypeStlLid:
				{
					STL_LID_RESULTS *pLR = (STL_LID_RESULTS *)pQR->QueryResult;
					STL_LID *pLid = pLR->Lids;
					for (i=0; i< pLR->NumLids; i++, pLid++) {
						pNR = ((STL_NODE_RECORD *)(GET_RESULT_OFFSET(pRsp, i)));
						*pLid = ntoh32(pNR->RID.LID);
					}
				}
				break;
			case OutputTypeStlNodeDesc:
				{
					STL_NODEDESC_RESULTS *pNDR = (STL_NODEDESC_RESULTS *)pQR->QueryResult;
					STL_NODE_DESCRIPTION *pDesc = pNDR->NodeDescs;
					for (i=0; i< pNDR->NumDescs; i++, pDesc++) {
						pNR = ((STL_NODE_RECORD *)(GET_RESULT_OFFSET(pRsp, i)));
						*pDesc = pNR->NodeDesc;
					}
				}
				break;
			}
		}
		break;

	case OutputTypePortInfoRecord:
		{
			PORTINFO_RECORD_RESULTS *pPIR;
			IB_PORTINFO_RECORD      *pPI = (IB_PORTINFO_RECORD *)mad.Data;
			int						extended_data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:     
				break;
			case InputTypeLid:           
				mad.SaHdr.ComponentMask = IB_PORTINFO_RECORD_COMP_ENDPORTLID;
				pPI->RID.s.EndPortLID = pQuery->InputValue.IbPortInfoRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port,"Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			BSWAP_IB_PORTINFO_RECORD(pPI, TRUE);
			MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_PORTINFO_RECORD);
			MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM, 
								 IB_SUBN_ADM_CLASS_VERSION);

			fstatus = sa_query_common(&mad, &pRsp, sizeof (IB_PORTINFO_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pPIR = (PORTINFO_RECORD_RESULTS*)pQR->QueryResult;
			pPI  = pPIR->PortInfoRecords;
			extended_data = ( (pRsp->SaHdr.AttributeOffset * 8) >=
							  sizeof(IB_PORTINFO_RECORD) );

			for (i=0; i< pPIR->NumPortInfoRecords; i++, pPI++) {
				*pPI =  * ((IB_PORTINFO_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_IB_PORTINFO_RECORD(pPI, extended_data);
				// Clear IB_LINK_SPEED_10G if QDR is overloaded
				if (extended_data) {
					if ( pPI->PortInfoData.CapabilityMask.s.IsExtendedSpeedsSupported &&
						 !( pPI->RID.s.Options &
							IB_PORTINFO_RECORD_OPTIONS_QDRNOTOVERLOADED ) ) {
						if (pPI->PortInfoData.LinkSpeedExt.Active)
							pPI->PortInfoData.LinkSpeed.Active &= ~IB_LINK_SPEED_10G;
						if (pPI->PortInfoData.LinkSpeedExt.Supported)
							pPI->PortInfoData.Link.SpeedSupported &= ~IB_LINK_SPEED_10G;
						if (pPI->PortInfoData.LinkSpeedExt.Enabled)
							pPI->PortInfoData.LinkSpeed.Enabled &= ~IB_LINK_SPEED_10G;
					}
				}
				else {
					pPI->PortInfoData.LinkSpeedExt.Active = 0;
					pPI->PortInfoData.LinkSpeedExt.Supported = 0;
					pPI->PortInfoData.LinkSpeedExt.Enabled = 0;
				}
			}
		}
		break;

	case OutputTypeStlPortInfoRecord:
		{
			STL_PORTINFO_RECORD_RESULTS	*pPIR;
			STL_PORTINFO_RECORD			*pPI = (STL_PORTINFO_RECORD *)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:     
				break;
			case InputTypeLid:           
				mad.SaHdr.ComponentMask = STL_PORTINFO_RECORD_COMP_ENDPORTLID;
				pPI->RID.EndPortLID = pQuery->InputValue.PortInfoRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port,"Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			BSWAP_STL_PORTINFO_RECORD(pPI);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_PORTINFO_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof (STL_PORTINFO_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pPIR = (STL_PORTINFO_RECORD_RESULTS*)pQR->QueryResult;
			pPI  = pPIR->PortInfoRecords;

			for (i=0; i< pPIR->NumPortInfoRecords; i++, pPI++) {
				*pPI =  * ((STL_PORTINFO_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_PORTINFO_RECORD(pPI);
			}
		}
		break;

	case OutputTypeStlLinkRecord:       
		{
			STL_LINK_RECORD_RESULTS *pLRR;
			STL_LINK_RECORD      *pLR = (STL_LINK_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:     
				break;
			case InputTypeLid:           
				mad.SaHdr.ComponentMask = IB_LINK_RECORD_COMP_FROMLID;
				pLR->RID.FromLID = pQuery->InputValue.PortInfoRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pLR->Reserved = 0;

			BSWAP_STL_LINK_RECORD(pLR);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_LINK_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof (STL_LINK_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pLRR = (STL_LINK_RECORD_RESULTS*)pQR->QueryResult;
			pLR = pLRR->LinkRecords;
			for (i=0; i< pLRR->NumLinkRecords; i++, pLR++) {
				*pLR =  * ((STL_LINK_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_LINK_RECORD(pLR);
			}
		}
		break;

	case OutputTypeStlSwitchInfoRecord:
    	{
			STL_SWITCHINFO_RECORD_RESULTS *pSIR = 0;
			STL_SWITCHINFO_RECORD	      *pSI = (STL_SWITCHINFO_RECORD*)mad.Data;
   
			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = STL_SWITCHINFO_RECORD_COMP_LID;
				pSI->RID.LID = pQuery->InputValue.SwitchInfoRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pSI->Reserved = 0;

			BSWAP_STL_SWITCHINFO_RECORD(pSI);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_SWITCHINFO_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_SWITCHINFO_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			pSIR = (STL_SWITCHINFO_RECORD_RESULTS*)pQR->QueryResult;
			pSI = pSIR->SwitchInfoRecords;
 
			for (i = 0; i < pSIR->NumSwitchInfoRecords; ++i, ++pSI) {
				*pSI = *((STL_SWITCHINFO_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_SWITCHINFO_RECORD(pSI);
			}
		}
		break;

	case OutputTypeStlSMInfoRecord:     
		{   
			STL_SMINFO_RECORD_RESULTS *pSMIR = 0;
			STL_SMINFO_RECORD      *pSMI = (STL_SMINFO_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:     
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pSMI->Reserved = 0;
                           
			BSWAP_STL_SMINFO_RECORD(pSMI);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_SMINFO_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof (STL_SMINFO_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pSMIR = (STL_SMINFO_RECORD_RESULTS*)pQR->QueryResult;
			pSMI  = pSMIR->SMInfoRecords;
			for (i=0; i< pSMIR->NumSMInfoRecords; i++, pSMI++) {
				*pSMI =  * ((STL_SMINFO_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_SMINFO_RECORD(pSMI);
			}
		}
		break;

	case OutputTypePathRecord:       
	case OutputTypePathRecordNetworkOrder:
		{   
			// Nota Bene: Path Records are different from other SA Queries
			// because there are two different queries that can result
			// in a list of Path Records. The first is an IB_PATH_RECORD
			// the second is an IB_MULTIPATH_RECORD.
			//
			// This slightly complicates the code in that the mad.Data
			// buffer may get cast as either record type, depending on 
			// the input arguments that were provided.
			
			PATH_RESULTS   *pPRR = 0;
			IB_PATH_RECORD *pPR = 0;
			IB_MULTIPATH_RECORD *pMPR = 0;
			uint16_t length = sizeof(IB_PATH_RECORD);
			int i;

			switch (pQuery->InputType) {
			case InputTypePortGuidList: 
				pMPR = (IB_MULTIPATH_RECORD *)mad.Data;
				MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_MULTIPATH_RECORD);
				MAD_SET_METHOD_TYPE (&mad, SUBN_ADM_GETMULTI);
				mad.SaHdr.ComponentMask = IB_MULTIPATH_RECORD_COMP_NUMBPATH |
					IB_MULTIPATH_RECORD_COMP_SGIDCOUNT |
					IB_MULTIPATH_RECORD_COMP_DGIDCOUNT; 
				pMPR->NumbPath = PATHRECORD_NUMBPATH;
				pMPR->SGIDCount = pQuery->InputValue.IbPathRecord.PortGuidList.SourceGuidCount;
				pMPR->DGIDCount = pQuery->InputValue.IbPathRecord.PortGuidList.DestGuidCount;
				for (i=0;i<(pMPR->SGIDCount+pMPR->DGIDCount); i++) {
					IB_GID *pGIDList = pMPR->GIDList; // fix compiler warning
					pGIDList[i].Type.Global.SubnetPrefix =
						pQuery->InputValue.IbPathRecord.PortGuidList.SharedSubnetPrefix;
					pGIDList[i].Type.Global.InterfaceID = 
						pQuery->InputValue.IbPathRecord.PortGuidList.GuidList[i];
				}
				length = sizeof(IB_MULTIPATH_RECORD) + 
					sizeof(IB_GID)*(pMPR->SGIDCount+pMPR->DGIDCount);
				BSWAP_IB_MULTIPATH_RECORD(pMPR);
				break;

			case InputTypeGidList: 
				pMPR = (IB_MULTIPATH_RECORD *)mad.Data;
				MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_MULTIPATH_RECORD);
				MAD_SET_METHOD_TYPE (&mad, SUBN_ADM_GETMULTI);
				mad.SaHdr.ComponentMask = IB_MULTIPATH_RECORD_COMP_NUMBPATH |
					IB_MULTIPATH_RECORD_COMP_SGIDCOUNT |
					IB_MULTIPATH_RECORD_COMP_DGIDCOUNT; 
				pMPR->NumbPath = PATHRECORD_NUMBPATH;
				pMPR->SGIDCount = pQuery->InputValue.IbPathRecord.GidList.SourceGidCount;
				pMPR->DGIDCount = pQuery->InputValue.IbPathRecord.GidList.DestGidCount;
				memcpy(pMPR->GIDList, 
					pQuery->InputValue.IbPathRecord.GidList.GidList,
					sizeof(IB_GID)*(pMPR->SGIDCount+pMPR->DGIDCount));
				length = sizeof(IB_MULTIPATH_RECORD) + 
					sizeof(IB_GID)*(pMPR->SGIDCount+pMPR->DGIDCount);
				BSWAP_IB_MULTIPATH_RECORD(pMPR);
				break;

			case InputTypeMultiPathRecord: 
				pMPR = (IB_MULTIPATH_RECORD *)mad.Data;
				MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_MULTIPATH_RECORD);
				MAD_SET_METHOD_TYPE (&mad, SUBN_ADM_GETMULTI);
				mad.SaHdr.ComponentMask = pQuery->InputValue.IbPathRecord.MultiPathRecord.ComponentMask;
				memcpy(pMPR,
					&pQuery->InputValue.IbPathRecord.MultiPathRecord.MultiPathRecord,
					sizeof(IB_MULTIPATH_RECORD));
				memcpy(pMPR->GIDList, 
					pQuery->InputValue.IbPathRecord.MultiPathRecord.Gids,
					sizeof(IB_GID)*(pMPR->SGIDCount+pMPR->DGIDCount));
				length = sizeof(IB_MULTIPATH_RECORD) + 
					sizeof(IB_GID)*(pMPR->SGIDCount+pMPR->DGIDCount);
				BSWAP_IB_MULTIPATH_RECORD(pMPR);
				break;

			case InputTypeNoInput:
			case InputTypeSourceGid:
				pPR = (IB_PATH_RECORD *)mad.Data;
				MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_PATH_RECORD);
				/* node record query??? */
				mad.SaHdr.ComponentMask = IB_PATH_RECORD_COMP_SGID |
					IB_PATH_RECORD_COMP_REVERSIBLE |
					IB_PATH_RECORD_COMP_NUMBPATH;
				pPR->SGID = pQuery->InputValue.IbPathRecord.SourceGid;
				pPR->Reversible                    = 1;
				pPR->NumbPath                      = PATHRECORD_NUMBPATH;
				BSWAP_IB_PATH_RECORD(pPR);
				break;


			case InputTypePKey:
				pPR = (IB_PATH_RECORD *)mad.Data;
				MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_PATH_RECORD);
				mad.SaHdr.ComponentMask = IB_PATH_RECORD_COMP_SGID |
					IB_PATH_RECORD_COMP_PKEY |
					IB_PATH_RECORD_COMP_REVERSIBLE |
					IB_PATH_RECORD_COMP_NUMBPATH;
				pPR->SGID = pQuery->InputValue.IbPathRecord.PKey.SourceGid;
				pPR->Reversible                    = 1;
				pPR->P_Key 		                   = pQuery->InputValue.IbPathRecord.PKey.PKey;
				pPR->NumbPath                      = PATHRECORD_NUMBPATH;
				BSWAP_IB_PATH_RECORD(pPR);
				break;

			case InputTypeSL:
				pPR = (IB_PATH_RECORD *)mad.Data;
				MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_PATH_RECORD);
				mad.SaHdr.ComponentMask = IB_PATH_RECORD_COMP_SGID |
					IB_PATH_RECORD_COMP_SL |
					IB_PATH_RECORD_COMP_REVERSIBLE |
					IB_PATH_RECORD_COMP_NUMBPATH;
				pPR->SGID = pQuery->InputValue.IbPathRecord.SL.SourceGid;
				pPR->Reversible                    = 1;
				pPR->u2.s.SL 	                   = pQuery->InputValue.IbPathRecord.SL.SL;
				pPR->NumbPath                      = PATHRECORD_NUMBPATH;
				BSWAP_IB_PATH_RECORD(pPR);
				break;

			case InputTypeServiceId:
				pPR = (IB_PATH_RECORD *)mad.Data;
				MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_PATH_RECORD);
				mad.SaHdr.ComponentMask = IB_PATH_RECORD_COMP_SGID |
					IB_PATH_RECORD_COMP_SERVICEID |
					IB_PATH_RECORD_COMP_REVERSIBLE |
					IB_PATH_RECORD_COMP_NUMBPATH;
				pPR->SGID = pQuery->InputValue.IbPathRecord.ServiceId.SourceGid;
				pPR->Reversible                    = 1;
				pPR->ServiceID	                   = pQuery->InputValue.IbPathRecord.ServiceId.ServiceId;
				pPR->NumbPath                      = PATHRECORD_NUMBPATH;
				BSWAP_IB_PATH_RECORD(pPR);
				break;

			case InputTypePortGuid:
			case InputTypePortGuidPair:
				pPR = (IB_PATH_RECORD *)mad.Data;
				MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_PATH_RECORD);
				mad.SaHdr.ComponentMask = IB_PATH_RECORD_COMP_DGID |
					IB_PATH_RECORD_COMP_SGID |
					IB_PATH_RECORD_COMP_REVERSIBLE |
					IB_PATH_RECORD_COMP_NUMBPATH;
				pPR->SGID.Type.Global.SubnetPrefix = pQuery->InputValue.IbPathRecord.PortGuid.SharedSubnetPrefix;
				pPR->DGID.Type.Global.SubnetPrefix = pQuery->InputValue.IbPathRecord.PortGuid.SharedSubnetPrefix;
				pPR->SGID.Type.Global.InterfaceID  = pQuery->InputValue.IbPathRecord.PortGuid.SourcePortGuid;
				pPR->DGID.Type.Global.InterfaceID  = pQuery->InputValue.IbPathRecord.PortGuid.DestPortGuid;
				pPR->Reversible                    = 1;
				pPR->NumbPath                      = PATHRECORD_NUMBPATH;
				BSWAP_IB_PATH_RECORD(pPR);
				break;

			case InputTypePortGid:
			case InputTypeGidPair:
				pPR = (IB_PATH_RECORD *)mad.Data;
				MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_PATH_RECORD);
				mad.SaHdr.ComponentMask = IB_PATH_RECORD_COMP_DGID |
					IB_PATH_RECORD_COMP_SGID |
					IB_PATH_RECORD_COMP_REVERSIBLE |
					IB_PATH_RECORD_COMP_NUMBPATH; 
				pPR->DGID = pQuery->InputValue.IbPathRecord.PortGid.DestGid;
				pPR->SGID = pQuery->InputValue.IbPathRecord.PortGid.SourceGid;
				pPR->Reversible = 1;
				pPR->NumbPath = PATHRECORD_NUMBPATH;
				BSWAP_IB_PATH_RECORD(pPR);
				break;
			case InputTypeLid:
				pPR = (IB_PATH_RECORD *)mad.Data;
				MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_PATH_RECORD);
				// This is going to be tricky.
				// I need to specify a source and destination for a path to be valid.
				// In this case - I have a dest lid, but my src lid is unknown to me.
				// BUT - I have my port lid and can create my port GID.
				// So - I'll use a gid for the source and a lid for dest....
				mad.SaHdr.ComponentMask = IB_PATH_RECORD_COMP_DLID |
					IB_PATH_RECORD_COMP_SGID |
					IB_PATH_RECORD_COMP_REVERSIBLE |
					IB_PATH_RECORD_COMP_NUMBPATH;
				pPR->SGID = pQuery->InputValue.IbPathRecord.Lid.SourceGid;
				pPR->DLID = pQuery->InputValue.IbPathRecord.Lid.DLid;
				pPR->Reversible                    = 1;
				pPR->NumbPath                      = PATHRECORD_NUMBPATH;
				BSWAP_IB_PATH_RECORD(pPR);
				break;

			case InputTypePathRecord:
			case InputTypePathRecordNetworkOrder:
				pPR = (IB_PATH_RECORD *)mad.Data;
				MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_PATH_RECORD);
				mad.SaHdr.ComponentMask = pQuery->InputValue.IbPathRecord.PathRecord.ComponentMask;
				*pPR = pQuery->InputValue.IbPathRecord.PathRecord.PathRecord;

				if (pQuery->InputType == InputTypePathRecord) {
					BSWAP_IB_PATH_RECORD(pPR);
				}
				break;

			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			if(pPR){
				pPR->u2.s.Reserved2 = 0;
				memset(pPR->Reserved2, 0, sizeof(pPR->Reserved2));
			}
			if(pMPR){
				pMPR->u2.s.Reserved2 = 0;
				pMPR->Reserved4= 0;
			}

			MAD_SET_VERSION_INFO(&mad, 
								 IB_BASE_VERSION, 
								 MCLASS_SUBN_ADM, 
								 IB_SUBN_ADM_CLASS_VERSION);

			fstatus = sa_query_common(&mad, &pRsp, length, &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pPRR = (PATH_RESULTS*)pQR->QueryResult;
			pPR  = pPRR->PathRecords;
			for (i=0; i< pPRR->NumPathRecords; i++, pPR++) {
				*pPR =  * ((IB_PATH_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				/* We do not want to swap the OutputTypePathRecordNetworkOrder type */
				if (pQuery->OutputType == OutputTypePathRecord) {
					BSWAP_IB_PATH_RECORD(pPR);
				}
			}
		}           
		break;

	case OutputTypeStlTraceRecord:   
        {   
			IB_PATH_RECORD				*pPR = (IB_PATH_RECORD*)mad.Data;
			STL_TRACE_RECORD_RESULTS 	*pTRR;
			STL_TRACE_RECORD			*pTR;

			switch (pQuery->InputType) {
			case InputTypePathRecord:
				mad.SaHdr.ComponentMask = pQuery->InputValue.TraceRecord.PathRecord.ComponentMask;
				*pPR = pQuery->InputValue.TraceRecord.PathRecord.PathRecord;
				break;
			case InputTypePortGuid:
			case InputTypePortGuidPair:
				mad.SaHdr.ComponentMask = IB_PATH_RECORD_COMP_DGID |
					IB_PATH_RECORD_COMP_SGID |
					IB_PATH_RECORD_COMP_REVERSIBLE |
					IB_PATH_RECORD_COMP_NUMBPATH;
				pPR->SGID.Type.Global.SubnetPrefix = pQuery->InputValue.TraceRecord.PortGuid.SharedSubnetPrefix;
				pPR->DGID.Type.Global.SubnetPrefix = pQuery->InputValue.TraceRecord.PortGuid.SharedSubnetPrefix;
				pPR->SGID.Type.Global.InterfaceID  = pQuery->InputValue.TraceRecord.PortGuid.SourcePortGuid;
				pPR->DGID.Type.Global.InterfaceID  = pQuery->InputValue.TraceRecord.PortGuid.DestPortGuid;
				pPR->Reversible                    = 1;
				pPR->NumbPath                      = PATHRECORD_NUMBPATH;
				break;
			case InputTypeGidPair:
			case InputTypePortGid:
				mad.SaHdr.ComponentMask = IB_PATH_RECORD_COMP_DGID |
					IB_PATH_RECORD_COMP_SGID |
					IB_PATH_RECORD_COMP_REVERSIBLE |
					IB_PATH_RECORD_COMP_NUMBPATH;
				pPR->SGID = pQuery->InputValue.TraceRecord.PortGid.SourceGid;
				pPR->DGID = pQuery->InputValue.TraceRecord.PortGid.DestGid;
				pPR->Reversible                    = 1;
				pPR->NumbPath                      = PATHRECORD_NUMBPATH;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}
                           
			BSWAP_IB_PATH_RECORD(pPR);
			MAD_SET_METHOD_TYPE (&mad, SUBN_ADM_GETTRACETABLE);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_TRACE_RECORD);
			
			fstatus = sa_query_common(&mad, &pRsp, sizeof(IB_PATH_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pTRR = (STL_TRACE_RECORD_RESULTS*)pQR->QueryResult;
			pTR  = pTRR->TraceRecords;
			for (i=0; i< pTRR->NumTraceRecords; i++, pTR++) {
				*pTR =  * ((STL_TRACE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				pTR->Reserved = 0;
				pTR->Reserved2 = 0;
				BSWAP_STL_TRACE_RECORD(pTR);
				pTR->NodeID ^= STL_TRACE_RECORD_COMP_ENCRYPT_MASK;
				pTR->ChassisID ^= STL_TRACE_RECORD_COMP_ENCRYPT_MASK;
				pTR->EntryPortID ^= STL_TRACE_RECORD_COMP_ENCRYPT_MASK;
				pTR->ExitPortID ^= STL_TRACE_RECORD_COMP_ENCRYPT_MASK;
			}
		}           
		break;

	case OutputTypeServiceRecord:   
		{   
			SERVICE_RECORD_RESULTS *pSRR;
			IB_SERVICE_RECORD      *pSR = (IB_SERVICE_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypePortGid:
				mad.SaHdr.ComponentMask = IB_SERVICE_RECORD_COMP_SERVICEGID;
				pSR->RID.ServiceGID = pQuery->InputValue.IbServiceRecord.ServiceGid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}
                           
			BSWAP_IB_SERVICE_RECORD(pSR);
			MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_SERVICE_RECORD);
			MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM, 
								 IB_SUBN_ADM_CLASS_VERSION);

			fstatus = sa_query_common(&mad, &pRsp, sizeof (IB_SERVICE_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pSRR = (SERVICE_RECORD_RESULTS*)pQR->QueryResult;
			pSR  = pSRR->ServiceRecords;
			for (i=0; i< pSRR->NumServiceRecords; i++, pSR++) {
				*pSR =  * ((IB_SERVICE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_IB_SERVICE_RECORD(pSR);
			}
		}
		break;

#ifndef NO_STL_MCMEMBER_OUTPUT      // Don't output STL McMember (use IB format) if defined
	case OutputTypeStlMcMemberRecord:  
        {   
			STL_MCMEMBER_RECORD_RESULTS *pMCRR;
			STL_MCMEMBER_RECORD	    *pMCR = (STL_MCMEMBER_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypePortGid:
				mad.SaHdr.ComponentMask = STL_MCMEMBER_COMPONENTMASK_PORTGID;
				pMCR->RID.PortGID = pQuery->InputValue.StlMcMemberRecord.PortGid;
				break;           
			case InputTypeMcGid:
				mad.SaHdr.ComponentMask = STL_MCMEMBER_COMPONENTMASK_MGID;
				pMCR->RID.MGID = pQuery->InputValue.StlMcMemberRecord.McGid;
				break;           
			case InputTypeLid:
				mad.SaHdr.ComponentMask = STL_MCMEMBER_COMPONENTMASK_MLID;
				pMCR->MLID = pQuery->InputValue.StlMcMemberRecord.Lid;
				break;
			case InputTypePKey:
				mad.SaHdr.ComponentMask = STL_MCMEMBER_COMPONENTMASK_PKEY;
				pMCR->P_Key = pQuery->InputValue.StlMcMemberRecord.PKey;
				break;
			case InputTypeSL:
                                mad.SaHdr.ComponentMask = STL_MCMEMBER_COMPONENTMASK_SL;
                                pMCR->SL = pQuery->InputValue.StlMcMemberRecord.SL;
                                break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pMCR->Reserved = 0;
			pMCR->Reserved2 = 0;
			pMCR->Reserved3 = 0;
			pMCR->Reserved4 = 0;
			pMCR->Reserved5 = 0;

			BSWAP_STL_MCMEMBER_RECORD(pMCR);
			MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_MCMEMBER_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof (STL_MCMEMBER_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pMCRR = (STL_MCMEMBER_RECORD_RESULTS*)pQR->QueryResult;
			pMCR  = pMCRR->McMemberRecords;
			for (i=0; i< pMCRR->NumMcMemberRecords; i++, pMCR++) {
				*pMCR =  * ((STL_MCMEMBER_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_MCMEMBER_RECORD(pMCR);
			}
		}
		break;
#endif

	case OutputTypeMcMemberRecord:
		{
			MCMEMBER_RECORD_RESULTS *pIbMCRR;
			IB_MCMEMBER_RECORD	    *pIbMCR = (IB_MCMEMBER_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypePortGid:
				mad.SaHdr.ComponentMask = IB_MCMEMBER_RECORD_COMP_PORTGID;
				pIbMCR->RID.PortGID = pQuery->InputValue.IbMcMemberRecord.PortGid;
				break;           
			case InputTypeMcGid:
				mad.SaHdr.ComponentMask = IB_MCMEMBER_RECORD_COMP_MGID;
				pIbMCR->RID.MGID = pQuery->InputValue.IbMcMemberRecord.McGid;
				break;           
			case InputTypeLid:
				mad.SaHdr.ComponentMask = IB_MCMEMBER_RECORD_COMP_MLID;
				pIbMCR->MLID = pQuery->InputValue.IbMcMemberRecord.Lid;
				break;
			case InputTypePKey:
				mad.SaHdr.ComponentMask = IB_MCMEMBER_RECORD_COMP_PKEY;
				pIbMCR->P_Key = pQuery->InputValue.IbMcMemberRecord.PKey;
				break;
			case InputTypeSL:
                                mad.SaHdr.ComponentMask = IB_MCMEMBER_RECORD_COMP_SL;
                                pIbMCR->u1.s.SL = pQuery->InputValue.IbMcMemberRecord.SL;
                                break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			BSWAP_IB_MCMEMBER_RECORD(pIbMCR);
			MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_MCMEMBER_RECORD);

			MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM,
								 IB_SUBN_ADM_CLASS_VERSION);

			fstatus = sa_query_common(&mad, &pRsp, sizeof (IB_MCMEMBER_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pIbMCRR = (MCMEMBER_RECORD_RESULTS*)pQR->QueryResult;
			pIbMCR  = pIbMCRR->McMemberRecords;
			for (i=0; i< pIbMCRR->NumMcMemberRecords; i++, pIbMCR++) {
				*pIbMCR =  * ((IB_MCMEMBER_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_IB_MCMEMBER_RECORD(pIbMCR);
			}
		}
		break;

	case OutputTypeInformInfoRecord:
		{   
			INFORM_INFO_RECORD_RESULTS *pIIRR;
			IB_INFORM_INFO_RECORD      *pIIR = (IB_INFORM_INFO_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypePortGid:
				mad.SaHdr.ComponentMask = IB_INFORMINFO_RECORD_COMP_SUBSCRIBERGID;
				pIIR->RID.SubscriberGID = pQuery->InputValue.IbInformInfoRecord.SubscriberGID;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			BSWAP_IB_INFORM_INFO_RECORD(pIIR);
			MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_INFORM_INFO_RECORD);
			MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM,
								 IB_SUBN_ADM_CLASS_VERSION);

			fstatus = sa_query_common(&mad, &pRsp, sizeof (IB_INFORM_INFO_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pIIRR = (INFORM_INFO_RECORD_RESULTS*)pQR->QueryResult;
			pIIR  = pIIRR->InformInfoRecords;
			for (i=0; i< pIIRR->NumInformInfoRecords; i++, pIIR++) {
				*pIIR =  * ((IB_INFORM_INFO_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_IB_INFORM_INFO_RECORD(pIIR);
			}
		}
		break;

	case OutputTypeStlInformInfoRecord:
		{   
			STL_INFORM_INFO_RECORD_RESULTS *pIIRR;
			STL_INFORM_INFO_RECORD      *pIIR = (STL_INFORM_INFO_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = STL_INFORM_INFO_REC_COMP_SUBSCRIBER_LID;
				pIIR->RID.SubscriberLID = pQuery->InputValue.StlInformInfoRecord.SubscriberLID;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pIIR->Reserved = 0;

			BSWAP_STL_INFORM_INFO_RECORD(pIIR);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_INFORM_INFO_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof (STL_INFORM_INFO_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pIIRR = (STL_INFORM_INFO_RECORD_RESULTS*)pQR->QueryResult;
			pIIR  = pIIRR->InformInfoRecords;
			for (i=0; i< pIIRR->NumInformInfoRecords; i++, pIIR++) {
				*pIIR =  * ((STL_INFORM_INFO_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_INFORM_INFO_RECORD(pIIR);
			}
		}
		break;
	case OutputTypeStlSCSCTableRecord:
		{
			STL_SC_MAPPING_TABLE_RECORD_RESULTS *pSCSCRR;
			STL_SC_MAPPING_TABLE_RECORD		*pSCSCR = (STL_SC_MAPPING_TABLE_RECORD*)mad.Data;

			pSCSCR->Reserved = 0;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = STL_SC2SC_RECORD_COMP_LID;
				pSCSCR->RID.LID = pQuery->InputValue.ScScTableRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			BSWAP_STL_SC_MAPPING_TABLE_RECORD(pSCSCR);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_SC_MAPTBL_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_SC_MAPPING_TABLE_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			pSCSCRR = (STL_SC_MAPPING_TABLE_RECORD_RESULTS*)pQR->QueryResult;
			pSCSCR  = pSCSCRR->SCSCRecords;
			for (i=0; i < pSCSCRR->NumSCSCTableRecords; i++, pSCSCR++) {
				*pSCSCR = *((STL_SC_MAPPING_TABLE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_SC_MAPPING_TABLE_RECORD(pSCSCR);
			}
		}
		break;

	case OutputTypeStlSLSCTableRecord:
		{	
			STL_SL2SC_MAPPING_TABLE_RECORD_RESULTS *pSLSCRR;
			STL_SL2SC_MAPPING_TABLE_RECORD *pSLSCR = (STL_SL2SC_MAPPING_TABLE_RECORD*)mad.Data;

			pSLSCR->RID.Reserved = 0;
			pSLSCR->Reserved2 = 0;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = STL_SL2SC_RECORD_COMP_LID; 
				pSLSCR->RID.LID = pQuery->InputValue.SlScTableRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER;
				goto done;
			}
			BSWAP_STL_SL2SC_MAPPING_TABLE_RECORD(pSLSCR);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_SL2SC_MAPTBL_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_SL2SC_MAPPING_TABLE_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) {
				break;
			}

			pSLSCRR = (STL_SL2SC_MAPPING_TABLE_RECORD_RESULTS*)pQR->QueryResult;
			pSLSCR = pSLSCRR->SLSCRecords;
			for (i=0; i < pSLSCRR->NumSLSCTableRecords; i++, pSLSCR++) {
				*pSLSCR = *((STL_SL2SC_MAPPING_TABLE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_SL2SC_MAPPING_TABLE_RECORD(pSLSCR);
			}
		}
		break;

	case OutputTypeStlSCSLTableRecord:
		{	
			STL_SC2SL_MAPPING_TABLE_RECORD_RESULTS *pSCSLRR;
			STL_SC2SL_MAPPING_TABLE_RECORD *pSCSLR = (STL_SC2SL_MAPPING_TABLE_RECORD*)mad.Data;

			pSCSLR->RID.Reserved = 0;
			pSCSLR->Reserved2 = 0;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = STL_SC2SL_RECORD_COMP_LID; 
				pSCSLR->RID.LID = pQuery->InputValue.ScSlTableRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER;
				goto done;
			}
			BSWAP_STL_SC2SL_MAPPING_TABLE_RECORD(pSCSLR);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_SC2SL_MAPTBL_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_SC2SL_MAPPING_TABLE_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) {
				break;
			}

			pSCSLRR = (STL_SC2SL_MAPPING_TABLE_RECORD_RESULTS*)pQR->QueryResult;
			pSCSLR = pSCSLRR->SCSLRecords;
			for (i=0; i < pSCSLRR->NumSCSLTableRecords; i++, pSCSLR++) {
				*pSCSLR = *((STL_SC2SL_MAPPING_TABLE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_SC2SL_MAPPING_TABLE_RECORD(pSCSLR);
			}
		}
		break;
	case OutputTypeStlSCVLtTableRecord:
		{
			STL_SC2PVL_T_MAPPING_TABLE_RECORD_RESULTS *pSCVLtRR;
			STL_SC2PVL_T_MAPPING_TABLE_RECORD *pSCVLtR = (STL_SC2PVL_T_MAPPING_TABLE_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = STL_SC2VL_R_RECORD_COMP_LID;
				pSCVLtR->RID.LID = pQuery->InputValue.ScVlxTableRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER;
				goto done;
			}
			BSWAP_STL_SC2VL_R_MAPPING_TABLE_RECORD(pSCVLtR);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_SC2VL_T_MAPTBL_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_SC2PVL_T_MAPPING_TABLE_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) {
				break;
			}

			pSCVLtRR = (STL_SC2PVL_T_MAPPING_TABLE_RECORD_RESULTS*)pQR->QueryResult;
			pSCVLtR = pSCVLtRR->SCVLtRecords;
			for (i=0; i < pSCVLtRR->NumSCVLtTableRecords; i++, pSCVLtR++) {
				*pSCVLtR = *((STL_SC2PVL_T_MAPPING_TABLE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_SC2VL_R_MAPPING_TABLE_RECORD(pSCVLtR);
			}
		}
		break;
	case OutputTypeStlSCVLntTableRecord:
		{
			STL_SC2PVL_NT_MAPPING_TABLE_RECORD_RESULTS *pSCVLntRR;
			STL_SC2PVL_NT_MAPPING_TABLE_RECORD *pSCVLntR = (STL_SC2PVL_NT_MAPPING_TABLE_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = STL_SC2VL_R_RECORD_COMP_LID;
				pSCVLntR->RID.LID = pQuery->InputValue.ScVlxTableRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER;
				goto done;
			}
			BSWAP_STL_SC2VL_R_MAPPING_TABLE_RECORD(pSCVLntR);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_SC2VL_NT_MAPTBL_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_SC2PVL_NT_MAPPING_TABLE_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) {
				break;
			}

			pSCVLntRR = (STL_SC2PVL_NT_MAPPING_TABLE_RECORD_RESULTS*)pQR->QueryResult;
			pSCVLntR = pSCVLntRR->SCVLntRecords;
			for (i=0; i < pSCVLntRR->NumSCVLntTableRecords; i++, pSCVLntR++) {
				*pSCVLntR = *((STL_SC2PVL_NT_MAPPING_TABLE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_SC2VL_R_MAPPING_TABLE_RECORD(pSCVLntR);
			}
		}
		break;
	case OutputTypeStlVLArbTableRecord: 
		{
			STL_VLARBTABLE_RECORD_RESULTS  *pVLRR;
			STL_VLARBTABLE_RECORD       *pVLR = (STL_VLARBTABLE_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = STL_VLARB_COMPONENTMASK_LID;
				pVLR->RID.LID = pQuery->InputValue.VlArbTableRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pVLR->Reserved = 0;

			BSWAP_STL_VLARBTABLE_RECORD(pVLR);

			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_VLARBTABLE_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof (STL_VLARBTABLE_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pVLRR = (STL_VLARBTABLE_RECORD_RESULTS*)pQR->QueryResult;
			pVLR  = pVLRR->VLArbTableRecords;
			for (i=0; i< pVLRR->NumVLArbTableRecords; i++, pVLR++) {
				*pVLR =  * ((STL_VLARBTABLE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_VLARBTABLE_RECORD(pVLR);
			}
		}
		break;

	case OutputTypeStlPKeyTableRecord:  
		{   
			STL_PKEYTABLE_RECORD_RESULTS  *pPKRR;
			STL_P_KEY_TABLE_RECORD     *pPKR = (STL_P_KEY_TABLE_RECORD*)mad.Data;

			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_P_KEY_TABLE_RECORD);

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = STL_PKEYTABLE_RECORD_COMP_LID;
				pPKR->RID.LID = pQuery->InputValue.PKeyTableRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pPKR->Reserved = 0;
			BSWAP_STL_PARTITION_TABLE_RECORD(pPKR);

			fstatus = sa_query_common(&mad, &pRsp, sizeof (STL_P_KEY_TABLE_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pPKRR = (STL_PKEYTABLE_RECORD_RESULTS*)pQR->QueryResult;
			pPKR  = pPKRR->PKeyTableRecords;
			for (i=0; i< pPKRR->NumPKeyTableRecords; i++, pPKR++) {
				*pPKR =  * ((STL_P_KEY_TABLE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_PARTITION_TABLE_RECORD(pPKR);
			}
		}
		break;

	case OutputTypeStlLinearFDBRecord:
		{
			STL_LINEAR_FDB_RECORD_RESULTS *pLFRR;
			STL_LINEAR_FORWARDING_TABLE_RECORD *pLFR = (STL_LINEAR_FORWARDING_TABLE_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = STL_LFT_RECORD_COMP_LID;
				pLFR->RID.LID = pQuery->InputValue.LinFdbTableRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pLFR->RID.Reserved = 0;

			BSWAP_STL_LINEAR_FORWARDING_TABLE_RECORD(pLFR);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_LINEAR_FWDTBL_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof (STL_LINEAR_FORWARDING_TABLE_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pLFRR = (STL_LINEAR_FDB_RECORD_RESULTS*)pQR->QueryResult;
			pLFR  = pLFRR->LinearFDBRecords;
			for (i=0; i< pLFRR->NumLinearFDBRecords; i++, pLFR++) {
				*pLFR =  * ((STL_LINEAR_FORWARDING_TABLE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_LINEAR_FORWARDING_TABLE_RECORD(pLFR);
			}
		}
		break;

	case OutputTypeStlMCastFDBRecord:
		{
			STL_MCAST_FDB_RECORD_RESULTS *pMFRR;
			STL_MULTICAST_FORWARDING_TABLE_RECORD *pMFR = (STL_MULTICAST_FORWARDING_TABLE_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = STL_MFTB_RECORD_COMP_LID;
				pMFR->RID.LID = pQuery->InputValue.McFdbTableRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pMFR->RID.u1.s.Reserved = 0;

			BSWAP_STL_MCFTB_RECORD(pMFR);
			MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_MCAST_FWDTBL_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof (IB_MCAST_FDB_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			//TODO: RESUME HERE (EEKAHN)
			// Translate the data.
			pMFRR = (STL_MCAST_FDB_RECORD_RESULTS*)pQR->QueryResult;
			pMFR  = pMFRR->MCastFDBRecords;
			for (i=0; i< pMFRR->NumMCastFDBRecords; i++, pMFR++) {
				*pMFR =  * ((STL_MULTICAST_FORWARDING_TABLE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_MCFTB_RECORD(pMFR);
			}
		}
		break;

	case OutputTypeStlVfInfoRecord:  
		{   
			STL_VFINFO_RECORD_RESULTS *pVFRR;
			STL_VFINFO_RECORD	*pVFR = (STL_VFINFO_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypePKey:
				mad.SaHdr.ComponentMask = STL_VFINFO_REC_COMP_PKEY;
				pVFR->pKey = pQuery->InputValue.VfInfoRecord.PKey;
				break;
			case InputTypeSL:
				mad.SaHdr.ComponentMask = STL_VFINFO_REC_COMP_SL;
				pVFR->s1.slBase = pQuery->InputValue.VfInfoRecord.SL;
				break;
			case InputTypeServiceId:
				mad.SaHdr.ComponentMask = STL_VFINFO_REC_COMP_SERVICEID;
				pVFR->ServiceID = pQuery->InputValue.VfInfoRecord.ServiceId;
				break;
			case InputTypeMcGid:
				mad.SaHdr.ComponentMask = STL_VFINFO_REC_COMP_MGID;
				pVFR->MGID = pQuery->InputValue.VfInfoRecord.McGid;
				break;           
			case InputTypeIndex:
				mad.SaHdr.ComponentMask = STL_VFINFO_REC_COMP_INDEX;
				pVFR->vfIndex = pQuery->InputValue.VfInfoRecord.vfIndex;
				break;
			case InputTypeNodeDesc:
				mad.SaHdr.ComponentMask = STL_VFINFO_REC_COMP_NAME;
				memcpy(pVFR->vfName, pQuery->InputValue.VfInfoRecord.vfName,
					STL_VFABRIC_NAME_LEN);
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pVFR->rsvd1 = 0;
			pVFR->s1.rsvd2 = 0;
			pVFR->s1.rsvd3 = 0;
			pVFR->s1.rsvd4 = 0;
			pVFR->s1.rsvd5 = 0;
			pVFR->rsvd6 = 0;
			pVFR->rsvd7 = 0;
			pVFR->rsvd8 = 0;
			pVFR->rsvd9 = 0;
			pVFR->rsvd10 = 0;
			memset(pVFR->rsvd11, 0, sizeof(pVFR->rsvd11));

			BSWAP_STL_VFINFO_RECORD(pVFR);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_VF_INFO_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_VFINFO_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			// Translate the data.
			pVFRR = (STL_VFINFO_RECORD_RESULTS*)pQR->QueryResult;
			pVFR  = pVFRR->VfInfoRecords;
			for (i=0; i< pVFRR->NumVfInfoRecords; i++, pVFR++) {
				*pVFR =  * ((STL_VFINFO_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_VFINFO_RECORD(pVFR);
			}
		}
		break;

	case OutputTypeStlFabricInfoRecord:
		{
			STL_FABRICINFO_RECORD_RESULT *pFIR;
			STL_FABRICINFO_RECORD *pFI;

			if (pQuery->InputType != InputTypeNoInput)
				break;

			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_FABRICINFO_RECORD);
			MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_GET);
			
			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_FABRICINFO_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) break;

			pFIR = (STL_FABRICINFO_RECORD_RESULT*)pQR->QueryResult;
			pFI = &pFIR->FabricInfoRecord;
			
			// There should only be one FabricInfoRecord result.
			if (pFIR->NumFabricInfoRecords > 0) {
				*pFI = *((STL_FABRICINFO_RECORD*)(GET_RESULT_OFFSET(pRsp, 0)));
				BSWAP_STL_FABRICINFO_RECORD(pFI);
			}
		}
		break;

	case OutputTypeStlQuarantinedNodeRecord:
		{
			STL_QUARANTINED_NODE_RECORD_RESULTS *pQNRR;
			STL_QUARANTINED_NODE_RECORD *pQNR = (STL_QUARANTINED_NODE_RECORD*)mad.Data;

			if(pQuery->InputType != InputTypeNoInput)
			{
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			BSWAP_STL_QUARANTINED_NODE_RECORD(pQNR);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_QUARANTINED_NODE_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_QUARANTINED_NODE_RECORD), &pQR, port);
			if(fstatus != FSUCCESS) break;

			// Translate the data
			pQNRR = (STL_QUARANTINED_NODE_RECORD_RESULTS*) pQR->QueryResult;
			pQNR = pQNRR->QuarantinedNodeRecords;

			for(i = 0; i < pQNRR->NumQuarantinedNodeRecords; i++, pQNR++)
			{
				*pQNR = * ((STL_QUARANTINED_NODE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));

				BSWAP_STL_QUARANTINED_NODE_RECORD(pQNR);
			}
			break;
		}

	case OutputTypeStlCongInfoRecord:
		{
			STL_CONGESTION_INFO_RECORD_RESULTS *pRecResults;
			STL_CONGESTION_INFO_RECORD *pRec = (STL_CONGESTION_INFO_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = CIR_COMPONENTMASK_COMP_LID;
				pRec->LID = pQuery->InputValue.CongInfoRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pRec->reserved = 0;

			BSWAP_STL_CONGESTION_INFO_RECORD(pRec);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_CONGESTION_INFO_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_CONGESTION_INFO_RECORD), &pQR, port);
			if(fstatus != FSUCCESS) break;

			// Translate the data
			pRecResults = (STL_CONGESTION_INFO_RECORD_RESULTS*)pQR->QueryResult;
			pRec = pRecResults->Records;

			for(i = 0; i < pRecResults->NumRecords; i++, pRec++)
			{
				*pRec = * ((STL_CONGESTION_INFO_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
                BSWAP_STL_CONGESTION_INFO_RECORD(pRec);
			}
			break;
		}

	case OutputTypeStlSwitchCongRecord:
		{
			STL_SWITCH_CONGESTION_SETTING_RECORD_RESULTS *pRecResults;
			STL_SWITCH_CONGESTION_SETTING_RECORD *pRec = (STL_SWITCH_CONGESTION_SETTING_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = SWCSR_COMPONENTMASK_COMP_LID;
				pRec->LID = pQuery->InputValue.SwCongRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pRec->reserved = 0;

			BSWAP_STL_SWITCH_CONGESTION_SETTING_RECORD(pRec);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_SWITCH_CONG_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_SWITCH_CONGESTION_SETTING_RECORD), &pQR, port);
			if(fstatus != FSUCCESS) break;

			// Translate the data
			pRecResults = (STL_SWITCH_CONGESTION_SETTING_RECORD_RESULTS*)pQR->QueryResult;
			pRec = pRecResults->Records;

			for(i = 0; i < pRecResults->NumRecords; i++, pRec++)
			{
				*pRec = * ((STL_SWITCH_CONGESTION_SETTING_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
                BSWAP_STL_SWITCH_CONGESTION_SETTING_RECORD(pRec);
			}
			break;
		}

	case OutputTypeStlSwitchPortCongRecord:
		{
			STL_SWITCH_PORT_CONGESTION_SETTING_RECORD_RESULTS *pRecResults;
			STL_SWITCH_PORT_CONGESTION_SETTING_RECORD *pRec = (STL_SWITCH_PORT_CONGESTION_SETTING_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = SWPCSR_COMPONENTMASK_COMP_LID;
				pRec->RID.LID = pQuery->InputValue.SwPortCongRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			memset(pRec->Reserved, 0, sizeof(pRec->Reserved));

			BSWAP_STL_SWITCH_PORT_CONGESTION_SETTING_RECORD(pRec);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_SWITCH_PORT_CONG_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_SWITCH_PORT_CONGESTION_SETTING_RECORD), &pQR, port);
			if(fstatus != FSUCCESS) break;

			// Translate the data
			pRecResults = (STL_SWITCH_PORT_CONGESTION_SETTING_RECORD_RESULTS*)pQR->QueryResult;
			pRec = pRecResults->Records;

			for(i = 0; i < pRecResults->NumRecords; i++, pRec++)
			{
				*pRec = * ((STL_SWITCH_PORT_CONGESTION_SETTING_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
                BSWAP_STL_SWITCH_PORT_CONGESTION_SETTING_RECORD(pRec);
			}
			break;
		}

	case OutputTypeStlHFICongRecord:
		{
			STL_HFI_CONGESTION_SETTING_RECORD_RESULTS *pRecResults;
			STL_HFI_CONGESTION_SETTING_RECORD *pRec = (STL_HFI_CONGESTION_SETTING_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = HCSR_COMPONENTMASK_COMP_LID;
				pRec->LID = pQuery->InputValue.HFICongRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pRec->reserved = 0;

			BSWAP_STL_HFI_CONGESTION_SETTING_RECORD(pRec);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_HFI_CONG_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_HFI_CONGESTION_SETTING_RECORD), &pQR, port);
			if(fstatus != FSUCCESS) break;

			// Translate the data
			pRecResults = (STL_HFI_CONGESTION_SETTING_RECORD_RESULTS*)pQR->QueryResult;
			pRec = pRecResults->Records;

			for(i = 0; i < pRecResults->NumRecords; i++, pRec++)
			{
				*pRec = * ((STL_HFI_CONGESTION_SETTING_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
                BSWAP_STL_HFI_CONGESTION_SETTING_RECORD(pRec);
			}
			break;
		}

	case OutputTypeStlHFICongCtrlRecord:
		{
			STL_HFI_CONGESTION_CONTROL_TABLE_RECORD_RESULTS *pRecResults;
			STL_HFI_CONGESTION_CONTROL_TABLE_RECORD *pRec = (STL_HFI_CONGESTION_CONTROL_TABLE_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = HCCTR_COMPONENTMASK_COMP_LID;
				pRec->RID.LID = pQuery->InputValue.HFICongCtrlRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pRec->reserved = 0;

			BSWAP_STL_HFI_CONGESTION_CONTROL_TABLE_RECORD(pRec);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_HFI_CONG_CTRL_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_RECORD), &pQR, port);
			if(fstatus != FSUCCESS) break;

			// Translate the data
			pRecResults = (STL_HFI_CONGESTION_CONTROL_TABLE_RECORD_RESULTS*)pQR->QueryResult;
			pRec = pRecResults->Records;
            
			for(i = 0; i < pRecResults->NumRecords; i++, pRec++)
			{
				*pRec = * ((STL_HFI_CONGESTION_CONTROL_TABLE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
                BSWAP_STL_HFI_CONGESTION_CONTROL_TABLE_RECORD(pRec);
			}
			break;
		}

    case OutputTypeStlBufCtrlTabRecord:
        {
			STL_BUFFER_CONTROL_TABLE_RECORD_RESULTS *pBCTRR;
			STL_BUFFER_CONTROL_TABLE_RECORD *pBCTR = (STL_BUFFER_CONTROL_TABLE_RECORD *)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = BFCTRL_COMPONENTMASK_COMP_LID;
				pBCTR->RID.LID = pQuery->InputValue.BufCtrlTableRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			memset(pBCTR->Reserved, 0, sizeof(pBCTR->Reserved));

			BSWAP_STL_BUFFER_CONTROL_TABLE_RECORD(pBCTR);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_BUFF_CTRL_TAB_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof (STL_BUFFER_CONTROL_TABLE_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) goto done;

			// Translate the data.
			pBCTRR = (STL_BUFFER_CONTROL_TABLE_RECORD_RESULTS*)pQR->QueryResult;
			pBCTR = pBCTRR->BufferControlRecords;
			for (i=0; i< pBCTRR->NumBufferControlRecords; i++, pBCTR++) {
				*pBCTR =  * ((STL_BUFFER_CONTROL_TABLE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_BUFFER_CONTROL_TABLE_RECORD(pBCTR);
			}
			break;
        }
	case OutputTypeStlCableInfoRecord:
		{
			STL_CABLE_INFO_RECORD_RESULTS *pCIRR;
			STL_CABLE_INFO_RECORD *pCIR = (STL_CABLE_INFO_RECORD *)mad.Data;
		
			switch (pQuery->InputType) {
				case InputTypeNoInput:
					break;
				case InputTypeLid:
					mad.SaHdr.ComponentMask |= STL_CIR_COMP_LID;
					pCIR->LID = pQuery->InputValue.CableInfoRecord.Lid;
					break;
				default:
					OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
							iba_sd_query_input_type_msg(pQuery->InputType),
							iba_sd_query_result_type_msg(pQuery->OutputType));
					fstatus = FINVALID_PARAMETER; goto done;
					break;
			}

			pCIR->Reserved = 0;

			// Default values.
			mad.SaHdr.ComponentMask |= STL_CIR_COMP_LEN | STL_CIR_COMP_ADDR;
			pCIR->Length = STL_CABLE_INFO_PAGESZ - 1;
			pCIR->u1.s.Address = STL_CIB_STD_HIGH_PAGE_ADDR;

			BSWAP_STL_CABLE_INFO_RECORD(pCIR);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_CABLE_INFO_RECORD);
			
			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_CABLE_INFO_RECORD), &pQR, port);
			if (fstatus != FSUCCESS) goto done;

			pCIRR = (STL_CABLE_INFO_RECORD_RESULTS *)pQR->QueryResult;
			pCIR = pCIRR->CableInfoRecords;
			for (i=0; i < pCIRR->NumCableInfoRecords; ++i, pCIR++) {
				*pCIR = *((STL_CABLE_INFO_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_CABLE_INFO_RECORD(pCIR);
			}
			break;
		}
    case OutputTypeStlPortGroupRecord:
		{
			STL_PORT_GROUP_TABLE_RECORD_RESULTS *pRecResults;
			STL_PORT_GROUP_TABLE_RECORD *pRec = (STL_PORT_GROUP_TABLE_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = STL_PGTB_RECORD_COMP_LID;
				pRec->RID.LID = pQuery->InputValue.PortGroupRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pRec->RID.Reserved = 0;
			pRec->Reserved2 = 0;

			BSWAP_STL_PORT_GROUP_TABLE_RECORD(pRec);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_PORTGROUP_TABLE_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_PORT_GROUP_TABLE_RECORD), &pQR, port);
			if(fstatus != FSUCCESS) break;

			// Translate the data
			pRecResults = (STL_PORT_GROUP_TABLE_RECORD_RESULTS*)pQR->QueryResult;
			pRec = pRecResults->Records;
            
			for(i = 0; i < pRecResults->NumRecords; i++, pRec++)
			{
				*pRec = * ((STL_PORT_GROUP_TABLE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
                BSWAP_STL_PORT_GROUP_TABLE_RECORD(pRec);
			}
			break;
		}
    case OutputTypeStlPortGroupFwdRecord:
		{
			STL_PORT_GROUP_FORWARDING_TABLE_RECORD_RESULTS *pRecResults;
			STL_PORT_GROUP_FORWARDING_TABLE_RECORD *pRec = (STL_PORT_GROUP_FORWARDING_TABLE_RECORD*)mad.Data;

			switch (pQuery->InputType) {
			case InputTypeNoInput:
				break;
			case InputTypeLid:
				mad.SaHdr.ComponentMask = STL_PGFWDTB_RECORD_COMP_LID;
				pRec->RID.LID = pQuery->InputValue.PortGroupFwdRecord.Lid;
				break;
			default:
				OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
						iba_sd_query_input_type_msg(pQuery->InputType),
						iba_sd_query_result_type_msg(pQuery->OutputType));
				fstatus = FINVALID_PARAMETER; goto done;
			}

			pRec->RID.u1.s.Reserved = 0;

			BSWAP_STL_PORT_GROUP_FORWARDING_TABLE_RECORD(pRec);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_PGROUP_FWDTBL_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_PORT_GROUP_FORWARDING_TABLE_RECORD), &pQR, port);
			if(fstatus != FSUCCESS) break;

			// Translate the data
			pRecResults = (STL_PORT_GROUP_FORWARDING_TABLE_RECORD_RESULTS*)pQR->QueryResult;
			pRec = pRecResults->Records;

			for(i = 0; i < pRecResults->NumRecords; i++, pRec++)
			{
				*pRec = * ((STL_PORT_GROUP_FORWARDING_TABLE_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
                BSWAP_STL_PORT_GROUP_FORWARDING_TABLE_RECORD(pRec);
			}
			break;
		}

	case OutputTypeStlSwitchCostRecord:
		{
			STL_SWITCH_COST_RECORD_RESULTS	*pSCR;
			STL_SWITCH_COST_RECORD			*pSC = (STL_SWITCH_COST_RECORD *)mad.Data;

			switch(pQuery->InputType) {
				case InputTypeNoInput:
					break;
				case InputTypeLid:
					mad.SaHdr.ComponentMask = STL_SWITCH_COST_REC_COMP_SLID;
					pSC->SLID = pQuery->InputValue.SwitchCostRecord.Lid;
					break;
				default:
					OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
							iba_sd_query_input_type_msg(pQuery->InputType),
							iba_sd_query_result_type_msg(pQuery->OutputType));
					fstatus = FINVALID_PARAMETER; goto done;
			}

			BSWAP_STL_SWITCH_COST_RECORD(pSC);
			MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_SWITCH_COST_RECORD);

			fstatus = sa_query_common(&mad, &pRsp, sizeof(STL_SWITCH_COST_RECORD), &pQR, port);
			if(fstatus != FSUCCESS) break;

			pSCR = (STL_SWITCH_COST_RECORD_RESULTS*)pQR->QueryResult;
			pSC = pSCR->Records;

			for(i = 0; i < pSCR->NumRecords; ++i, ++pSC)
			{
				*pSC = *((STL_SWITCH_COST_RECORD*)(GET_RESULT_OFFSET(pRsp, i)));
				BSWAP_STL_SWITCH_COST_RECORD(pSC);
			}
			break;
		}

	default:
		OMGT_OUTPUT_ERROR(port, "Query not supported by opamgt: Input=%s, Output=%s\n",
				iba_sd_query_input_type_msg(pQuery->InputType),
				iba_sd_query_result_type_msg(pQuery->OutputType));
		fstatus = FINVALID_PARAMETER; goto done;
		break;
	}

done:
	// Common place to print error for PKEY mismatch
	if (fstatus == FPROTECTION) {
		OMGT_OUTPUT_ERROR(port, "Unable to send query, requires full management PKEY\n");
	}
	if (pRsp!=NULL) {
		free (pRsp);
	}
	pRsp = NULL;
	*ppQueryResult = pQR;

	DBG_EXIT_FUNC(port);

	return fstatus;
}

/* omgt_query_sa
 *
 * All SA queries are sent through this path. This function performs a test of
 * SA reachability before sending the main query. If the FM's reachability is
 * not operational, a request for a ClassPortInfo with a small timeout
 * (defaulted to 250ms in lower layer) is sent. Only after determining the SA
 * is reachable is the main query sent. All other queries currently default to
 * a timeout of 20s with 3 retires. Sending the CPI first allows us to do a
 * quick ping and avoid long timeouts when SA is unreachable.
 *
 * if pQuery and ppQueryResult are input NULL, only a reachability test will be
 * performed with no main query.
 *
 */
FSTATUS omgt_query_sa(struct omgt_port *port, OMGT_QUERY *pQuery,
	struct _QUERY_RESULT_VALUES **ppQueryResult)
{
	FSTATUS fstatus = FSUCCESS;

	if (port == NULL)
		return FINVALID_PARAMETER;

	if (port->sa_service_state != OMGT_SERVICE_STATE_OPERATIONAL && !port->is_oob_enabled) {
		QUERY_RESULT_VALUES *cpi_query_result = NULL;
		OMGT_QUERY cpi_query;
		cpi_query.InputType = InputTypeNoInput;
		cpi_query.OutputType = OutputTypeStlClassPortInfo;
		fstatus = omgt_query_sa_internal(port, &cpi_query, &cpi_query_result);

		if (fstatus == FSUCCESS) {
			port->sa_service_state = OMGT_SERVICE_STATE_OPERATIONAL;
			if (pQuery != NULL && ppQueryResult != NULL &&
				pQuery->OutputType == OutputTypeStlClassPortInfo &&
				pQuery->InputType == InputTypeNoInput && cpi_query_result != NULL) {
				/* If request is a ClassPortInfo Query, then just return with results */
				*ppQueryResult = cpi_query_result;
				return FSUCCESS;
			}
		} else {
			OMGT_OUTPUT_ERROR(port, "SA Service State refresh failed: %u. Marking SA and PA Service State DOWN\n", fstatus);
			port->sa_service_state = OMGT_SERVICE_STATE_DOWN;
			/* If SA is down assume PA is down */
			port->pa_service_state = OMGT_SERVICE_STATE_DOWN;
		}
		omgt_free_query_result_buffer(cpi_query_result);
	}

	if (fstatus == FSUCCESS && pQuery != NULL && ppQueryResult != NULL) {
		fstatus = omgt_query_sa_internal(port, pQuery, ppQueryResult); 
		if (fstatus != FSUCCESS) {
			OMGT_OUTPUT_ERROR(port, "Query Failed: %u. Marking SA and PA Service State DOWN\n", fstatus);
			port->sa_service_state = OMGT_SERVICE_STATE_DOWN;
			/* If SA is down assume PA is down */
			port->pa_service_state = OMGT_SERVICE_STATE_DOWN;
		}
	}

	return fstatus;
}
