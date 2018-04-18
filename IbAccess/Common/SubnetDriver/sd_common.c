/* BEGIN_ICS_COPYRIGHT6 ****************************************

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

** END_ICS_COPYRIGHT6   ****************************************/

#include "stl_sd.h"

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

const char* iba_sd_query_input_type_msg(QUERY_INPUT_TYPE code)
{
	if (code < 0 || code >= (int)(sizeof(SdQueryInputTypeText)/sizeof(char*)))
		return "Unknown SD Query Input Type";
	else
		return SdQueryInputTypeText[code];
}

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
	"OutputTypeVfInfoRecord",			/* VF_RECORD_RESULTS is set of VF info records */
    // PM query results
	"OutputTypePaRecord",             /* PA_PACKET_RESULTS complete PA SinglePacketRespRecords */
	"OutputTypePaTableRecord",        /* PA_TABLE_PACKET_RESULTS complete PA MultiPacketRespRecords */
};

const char* iba_sd_query_result_type_msg(QUERY_RESULT_TYPE code)
{
	if (code < 0 || code >= (int)(sizeof(SdQueryResultTypeText)/sizeof(char*)))
		return "Unknown SD Query Result Type";
	else
		return SdQueryResultTypeText[code];
}

static const char* const SdFabricOpTypeText[] = {
	"FabOpSetServiceRecord",
	"FabOpDeleteServiceRecord",
	"FabOpSetMcMemberRecord",
	"FabOpJoinMcGroup",
	"FabOpLeaveMcGroup",
	"FabOpDeleteMcMemberRecord",
	"FabOpSetInformInfo",
};

const char* iba_sd_fabric_op_type_msg(FABRIC_OPERATION_TYPE code)
{
	if (code < 0 || code >= (int)(sizeof(SdFabricOpTypeText)/sizeof(char*)))
		return "Unknown SD Fabric Op Type";
	else
		return SdFabricOpTypeText[code];
}

// upper 8 bits of MAD_STATUS_SA_* fields from ib_sa_records.h
static const char* const SdSAStatusText[] = {
	"Success",	// not used by code below
	"Insufficient SA Resources",
	"Invalid SA Request",
	"No SA Records",
	"Too Many SA Records",
	"Invalid GID in SA Request",
	"Insufficient Components in SA Request"
};

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
