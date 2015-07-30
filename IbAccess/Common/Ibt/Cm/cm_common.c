/* BEGIN_ICS_COPYRIGHT6 ****************************************

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

** END_ICS_COPYRIGHT6   ****************************************/

// Common code between user and kernel level CM
#include "ib_cm.h"

// backwards compatibilty functions
IB_HANDLE
CmCreateCEP(
	CM_CEP_TYPE	TransportServiceType
	)
{
	return iba_cm_create_cep(TransportServiceType);
}

FSTATUS
CmDestroyCEP(IB_HANDLE hCEP)
{
	return iba_cm_destroy_cep(hCEP);
}

FSTATUS
CmModifyCEP(
	IB_HANDLE		hCEP,
	uint32			AttrType,
	const char*		AttrValue,
	uint32			AttrLen,
	uint32			Offset
	)
{
	return iba_cm_modify_cep( hCEP, AttrType, AttrValue, AttrLen, Offset);
}

FSTATUS
CmConnect(
	IB_HANDLE				hCEP,
	const CM_REQUEST_INFO*	pConnectRequest,
	PFN_CM_CALLBACK			pfnConnectCB,
	void*					Context
	)
{
	return iba_cm_connect(hCEP, pConnectRequest, pfnConnectCB, Context);
}

FSTATUS
CmConnectPeer(
	IB_HANDLE				hCEP,
	const CM_REQUEST_INFO*	pConnectRequest,
	PFN_CM_CALLBACK			pfnConnectCB,
	void*					Context
	)
{
	return iba_cm_connect_peer( hCEP, pConnectRequest, pfnConnectCB, Context);
}


FSTATUS
CmListen(
	IN IB_HANDLE				hCEP,
	IN const CM_LISTEN_INFO*	pListenInfo,
	IN PFN_CM_CALLBACK			pfnListenCB,
	IN void*					Context
	)
{
	return iba_cm_listen( hCEP, pListenInfo, pfnListenCB, Context);
}


FSTATUS
CmWait(
	IB_HANDLE				CEPHandleArray[],
	CM_CONN_INFO*			ConnInfoArray[],
	uint32					ArrayCount,
	uint32					Timeout_us
	//EVENT_HANDLE			hWaitEvent		// Associate the array of handles to this event
	)
{
	return iba_cm_wait(CEPHandleArray, ConnInfoArray, ArrayCount, Timeout_us);
}


FSTATUS
CmAccept(
	IN IB_HANDLE				hCEP,
	IN CM_CONN_INFO*			pSendConnInfo,		// Send REP
	OUT CM_CONN_INFO*			pRecvConnInfo,		// Rcvd RTU, REJ or TIMEOUT
	IN PFN_CM_CALLBACK			pfnCallback,
	IN void*					Context,
	OUT IB_HANDLE*				hNewCEP
	)
{
	return iba_cm_accept( hCEP, pSendConnInfo, pRecvConnInfo, pfnCallback,
						Context, hNewCEP);
}



FSTATUS
CmReject(
	IN IB_HANDLE				hCEP,
	IN const CM_REJECT_INFO*	pConnectReject
	)
{
	return iba_cm_reject( hCEP, pConnectReject);
}

FSTATUS
CmCancel(
	IN IB_HANDLE				hCEP
	)
{
	return iba_cm_cancel( hCEP);
}


FSTATUS
CmDisconnect(
	IN IB_HANDLE				hCEP,
	IN const CM_DREQUEST_INFO*	pDRequest,	// Send DREQ
	IN const CM_DREPLY_INFO*	pDReply	// Send DREP
	)
{
	return iba_cm_disconnect( hCEP, pDRequest, pDReply);
}


FSTATUS
SIDRRegister(
	IN IB_HANDLE				hCEP,
	IN const SIDR_REGISTER_INFO*		pSIDRRegisterInfo,
	PFN_CM_CALLBACK				pfnLookupCallback,
	void*						Context
	)
{
	return iba_cm_sidr_register( hCEP, pSIDRRegisterInfo, pfnLookupCallback,
							Context);
}


FSTATUS
SIDRDeregister(
	IB_HANDLE		hCEP
	)
{
	return iba_cm_sidr_deregister(hCEP);
}


FSTATUS
SIDRResponse(
	IN IB_HANDLE			hCEP,
	const SIDR_RESP_INFO*	pSIDRResponse
	)
{
	return iba_cm_sidr_response( hCEP, pSIDRResponse);
}

FSTATUS
SIDRQuery(
	IB_HANDLE			hCEP,
	const SIDR_REQ_INFO* pSIDRRequest,
	PFN_CM_CALLBACK		pfnQueryCB,
	void*				Context
	)
{
	return iba_cm_sidr_query( hCEP, pSIDRRequest, pfnQueryCB, Context);
}

static const char* const CmRejectionCodeText[] = {
	"Unknown CM Rejection Code",
	"RC_NO_QP",
	"RC_NO_EEC",
	"RC_NO_RESOURCES",
	"RC_TIMEOUT",
	"RC_UNSUPPORTED_REQ",
	"RC_INVALID_COMMID",
	"RC_INVALID_COMMINST",
	"RC_INVALID_SID",
	"RC_INVALID_TSTYPE",
	"RC_STALE_CONN",
	"RC_INVALID_RDC",
	"RC_PRIMARY_DGID_REJ",
	"RC_PRIMARY_DLID_REJ",
	"RC_INVALID_PRIMARY_SL",
	"RC_INVALID_PRIMARY_TC",
	"RC_INVALID_PRIMARY_HL",
	"RC_INVALID_PRIMARY_PR",
	"RC_ALTERNATE_DGID",
	"RC_ALTERNATE_DLID",
	"RC_INVALID_ALTERNATE_SL",
	"RC_INVALID_ALTERNATE_TC",
	"RC_INVALID_ALTERNATE_HL",
	"RC_INVALID_ALTERNATE_PR",
	"RC_CMPORT_REDIR",
	"RC_INVALID_PATHMTU",
	"RC_INSUFFICIENT_RESP_RES",
	"RC_USER_REJ",
	"RC_RNRCOUNT_REJ"
};

const char* iba_cm_rejection_code_msg(uint16 code)
{
	if (code >= (unsigned)(sizeof(CmRejectionCodeText)/sizeof(char*)))
		return "Unknown CM Rejection Code";
	else
		return CmRejectionCodeText[code];
}

static const char* const CmAprStatusText[] = {
	"APS_PATH_LOADED",
	"APS_INVALID_COMMID",
	"APS_UNSUPPORTED_REQ",
	"APS_REJECTED",
	"APS_CMPORT_REDIR",
	"APS_DUPLICATE_PATH",
	"APS_ENDPOINT_MISMATCH",
	"APS_REJECT_DLID",
	"APS_REJECT_DGID",
	"APS_REJECT_FL",
	"APS_REJECT_TC",
	"APS_REJECT_HL",
	"APS_REJECT_PR",
	"APS_REJECT_SL"
};

const char* iba_cm_apr_status_msg(uint16 code)
{
	if (code >= (unsigned)(sizeof(CmAprStatusText)/sizeof(char*)))
		return "Unknown CM APS Status";
	else
		return CmAprStatusText[code];
}

static const char* const CmSidrRespStatusText[] = {
	"SRS_VALID_QPN",
	"SRS_SID_NOT_SUPPORTED",
	"SRS_S_PROVIDER_REJECTED",
	"SRS_QP_UNAVAILABLE",
	"SRS_REDIRECT",
	"SRS_VERSION_NOT_SUPPORTED"
	// 6-255 reserved
};

const char* iba_cm_sidr_resp_status_msg(uint8 code)
{
	if (code >= (unsigned)(sizeof(CmSidrRespStatusText)/sizeof(char*)))
		return "Unknown CM SIDR Resp Status";
	else
		return CmSidrRespStatusText[code];
}

static const char* const CmRepFailoverText[] = {
	"CM_REP_FO_ACCEPTED",
	"CM_REP_FO_NOT_SUPPORTED",
	"CM_REP_FO_REJECTED_ALT"
};

const char* iba_cm_rep_failover_msg(uint8 code)
{
	if (code >= (unsigned)(sizeof(CmRepFailoverText)/sizeof(char*)))
		return "Unknown CM Failover Reply";
	else
		return CmRepFailoverText[code];
}
