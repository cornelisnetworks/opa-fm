/* BEGIN_ICS_COPYRIGHT4 ****************************************

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

** END_ICS_COPYRIGHT4   ****************************************/

// Public header file
#include "ib_cm.h"

// Private header file
#include "cm_private.h"


//////////////////////////////////////////////////////////////////////////
// FormatClassPortInfo
//
// Format the ClassPortInfo message
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	None.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//

void
FormatClassPortInfo(CM_MAD* Mad,
		  uint64 TransactionID)
{
	// Set up the mad header
	MAD_SET_VERSION_INFO(Mad, IB_BASE_VERSION, MCLASS_COMM_MGT, IB_COMM_MGT_CLASS_VERSION);
	
	MAD_SET_ATTRIB_ID(Mad, MCLASS_ATTRIB_ID_CLASS_PORT_INFO);

	Mad->common.mr.AsReg8 = MMTHD_GET_RESP;

	MAD_SET_TRANSACTION_ID(Mad, TransactionID);

	// We do not implement GSI Redirection nor Traps for this class
	MemoryClear(&Mad->payload, sizeof(Mad->payload));
	Mad->payload.ClassPortInfo.BaseVersion = IB_BASE_VERSION;
	Mad->payload.ClassPortInfo.ClassVersion = IB_COMM_MGT_CLASS_VERSION;
	Mad->payload.ClassPortInfo.CapMask = 0;
	Mad->payload.ClassPortInfo.u1.s.RespTimeValue = PORT_RESP_TIME_VALUE_MAX; // 8
	DUMP_CLASS_PORT_INFO(&Mad->payload.ClassPortInfo);

	ConvertMadToNetworkByteOrder(Mad);

} // FormatREQ()


//////////////////////////////////////////////////////////////////////////
// FormatREQ
//
// Format the REQ message
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	None.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//

void
FormatREQ(CM_MAD* Mad, 
		  const CM_REQUEST_INFO* pConnectRequest,
		  uint64 TransactionID,
		  uint32 LocalCommID,
		  uint32 CMQKey,
		  CM_CEP_TYPE Type,
		  uint32 LocalCMTimeout, 
		  uint32 RemoteCMTimeout, 
		  uint32 MaxCMRetries,
		  uint8 OfferedInitiatorDepth,
		  uint8 OfferedResponderResources)
{
	// Set up the mad header
	MAD_SET_VERSION_INFO(Mad, IB_BASE_VERSION, MCLASS_COMM_MGT, IB_COMM_MGT_CLASS_VERSION);
	
	MAD_SET_ATTRIB_ID(Mad, MCLASS_ATTRIB_ID_REQ); // Connection request msg

	MAD_SET_METHOD_TYPE(Mad, MMTHD_SEND);

	MAD_SET_TRANSACTION_ID(Mad, TransactionID);

	// Set up the REQ msg
	Mad->payload.REQ.LocalCommID				= LocalCommID;
	Mad->payload.REQ.Reserved1					= 0;
	Mad->payload.REQ.ServiceID					= pConnectRequest->SID;
	Mad->payload.REQ.LocalCaGUID				= pConnectRequest->CEPInfo.CaGUID;
	Mad->payload.REQ.Reserved2					= 0;
	Mad->payload.REQ.LocalQKey					= pConnectRequest->CEPInfo.QKey;
	Mad->payload.REQ.u1.s.LocalQPN				= pConnectRequest->CEPInfo.QPN;
	Mad->payload.REQ.u1.s.OfferedResponderResources	= OfferedResponderResources;
	Mad->payload.REQ.u2.s.LocalEECN				= pConnectRequest->CEPInfo.LocalEECN;
	Mad->payload.REQ.u2.s.OfferedInitiatorDepth	= OfferedInitiatorDepth;
	Mad->payload.REQ.u3.s.RemoteEECN			= pConnectRequest->CEPInfo.RemoteEECN;
	Mad->payload.REQ.u3.s.RemoteCMTimeout		= RemoteCMTimeout;
	Mad->payload.REQ.u3.s.TransportServiceType	= (uint8)Type;
	Mad->payload.REQ.u3.s.EndToEndFlowControl	= pConnectRequest->CEPInfo.EndToEndFlowControl;
	Mad->payload.REQ.u4.s.StartingPSN			= pConnectRequest->CEPInfo.StartingPSN;
	Mad->payload.REQ.u4.s.LocalCMTimeout		= LocalCMTimeout;
	Mad->payload.REQ.u4.s.RetryCount			= pConnectRequest->CEPInfo.RetryCount;
	Mad->payload.REQ.PartitionKey				= pConnectRequest->PathInfo.Path.P_Key;
	Mad->payload.REQ.PathMTU					= pConnectRequest->PathInfo.Path.Mtu;
	if (pConnectRequest->CEPInfo.RemoteEECN)
		Mad->payload.REQ.RdcExists				= 1;
	else
		Mad->payload.REQ.RdcExists				= 0;

	Mad->payload.REQ.RnRRetryCount				= (uint16)pConnectRequest->CEPInfo.RnrRetryCount;
	Mad->payload.REQ.MaxCMRetries				= (uint16)MaxCMRetries;
	Mad->payload.REQ.Reserved3					= 0;

	Mad->payload.REQ.PrimaryLocalLID			= pConnectRequest->PathInfo.Path.SLID;
	Mad->payload.REQ.PrimaryRemoteLID			= pConnectRequest->PathInfo.Path.DLID;
	Mad->payload.REQ.PrimaryLocalGID			= pConnectRequest->PathInfo.Path.SGID;
	Mad->payload.REQ.PrimaryRemoteGID			= pConnectRequest->PathInfo.Path.DGID;

	Mad->payload.REQ.u5.s.PrimaryFlowLabel		= pConnectRequest->PathInfo.Path.u1.s.FlowLabel;
	Mad->payload.REQ.u5.s.Reserved4				= 0;
	Mad->payload.REQ.u5.s.PrimaryPacketRate		= pConnectRequest->PathInfo.Path.Rate;
	Mad->payload.REQ.PrimaryTrafficClass		= pConnectRequest->PathInfo.Path.TClass;
	Mad->payload.REQ.PrimaryHopLimit			= pConnectRequest->PathInfo.Path.u1.s.HopLimit;
	Mad->payload.REQ.PrimarySL					= pConnectRequest->PathInfo.Path.u2.s.SL;
	Mad->payload.REQ.PrimarySubnetLocal			= pConnectRequest->PathInfo.bSubnetLocal;
	Mad->payload.REQ.Reserved5					= 0;
	Mad->payload.REQ.PrimaryLocalAckTimeout		= pConnectRequest->CEPInfo.AckTimeout;
	Mad->payload.REQ.Reserved6					= 0;


	if (pConnectRequest->AlternatePathInfo.Path.SLID) // Alternate path is specified
	{
		Mad->payload.REQ.AlternateLocalLID			= pConnectRequest->AlternatePathInfo.Path.SLID;
		Mad->payload.REQ.AlternateRemoteLID			= pConnectRequest->AlternatePathInfo.Path.DLID;
		Mad->payload.REQ.AlternateLocalGID			= pConnectRequest->AlternatePathInfo.Path.SGID;
		Mad->payload.REQ.AlternateRemoteGID			= pConnectRequest->AlternatePathInfo.Path.DGID;

		Mad->payload.REQ.u6.s.AlternateFlowLabel	= pConnectRequest->AlternatePathInfo.Path.u1.s.FlowLabel;
		Mad->payload.REQ.u6.s.Reserved7				= 0;
		Mad->payload.REQ.u6.s.AlternatePacketRate	= pConnectRequest->AlternatePathInfo.Path.Rate;
		Mad->payload.REQ.AlternateTrafficClass		= pConnectRequest->AlternatePathInfo.Path.TClass;
		Mad->payload.REQ.AlternateHopLimit			= pConnectRequest->AlternatePathInfo.Path.u1.s.HopLimit;
		Mad->payload.REQ.AlternateSL				= pConnectRequest->AlternatePathInfo.Path.u2.s.SL;

		Mad->payload.REQ.AlternateSubnetLocal		= pConnectRequest->AlternatePathInfo.bSubnetLocal;
		Mad->payload.REQ.Reserved8					= 0;
		Mad->payload.REQ.AlternateLocalAckTimeout	= pConnectRequest->CEPInfo.AlternateAckTimeout;
		Mad->payload.REQ.Reserved9					= 0;
	}
	else
	{
		Mad->payload.REQ.AlternateLocalLID			= 0;
		Mad->payload.REQ.AlternateRemoteLID			= 0;
		MemoryClear(&Mad->payload.REQ.AlternateLocalGID.Raw, sizeof(IB_GID));
		MemoryClear(&Mad->payload.REQ.AlternateRemoteGID.Raw, sizeof(IB_GID));

		Mad->payload.REQ.u6.s.AlternateFlowLabel	= 0;
		Mad->payload.REQ.u6.s.Reserved7				= 0;
		Mad->payload.REQ.u6.s.AlternatePacketRate	= 0;
		Mad->payload.REQ.AlternateTrafficClass		= 0;
		Mad->payload.REQ.AlternateHopLimit			= 0;
		Mad->payload.REQ.AlternateSL				= 0;
		Mad->payload.REQ.AlternateSubnetLocal		= 0;
		Mad->payload.REQ.Reserved8					= 0;
		Mad->payload.REQ.AlternateLocalAckTimeout	= 0;
		Mad->payload.REQ.Reserved9					= 0;
	}

	memcpy(Mad->payload.REQ.PrivateData, pConnectRequest->PrivateData, CMM_REQ_USER_LEN);

	DUMP_REQ(&Mad->payload.REQ);

	ConvertMadToNetworkByteOrder(Mad);

} // FormatREQ()


void
CopyREQToReqInfo(
	const CMM_REQ* pREQ,
	CM_REQUEST_INFO *pConnectRequest)
{
	// Primary path info
	pConnectRequest->PathInfo.bSubnetLocal				= (boolean)pREQ->PrimarySubnetLocal;

	pConnectRequest->PathInfo.Path.SLID					= pREQ->PrimaryRemoteLID;
	pConnectRequest->PathInfo.Path.SGID					= pREQ->PrimaryRemoteGID;

	pConnectRequest->PathInfo.Path.DLID					= pREQ->PrimaryLocalLID;
	pConnectRequest->PathInfo.Path.DGID					= pREQ->PrimaryLocalGID;

	pConnectRequest->PathInfo.Path.NumbPath				= 1; // Only one path.

	pConnectRequest->PathInfo.Path.u1.s.FlowLabel		= pREQ->u5.s.PrimaryFlowLabel;
	pConnectRequest->PathInfo.Path.u1.s.HopLimit		= pREQ->PrimaryHopLimit;
	pConnectRequest->PathInfo.Path.TClass				= (uint8)pREQ->PrimaryTrafficClass;
	pConnectRequest->PathInfo.Path.u2.s.SL				= (uint8)pREQ->PrimarySL;
	pConnectRequest->PathInfo.Path.Rate					= (uint8)pREQ->u5.s.PrimaryPacketRate;
	pConnectRequest->PathInfo.Path.RateSelector			= IB_SELECTOR_EQ;

	pConnectRequest->PathInfo.Path.P_Key				= pREQ->PartitionKey;
	pConnectRequest->PathInfo.Path.Mtu					= (uint8)pREQ->PathMTU;
	
	pConnectRequest->PathInfo.Path.MtuSelector			= IB_SELECTOR_EQ;
			
	// since exponent, subtracting 1 means divide by 2
	// this value ends up rounded up by Ca Ack Delay/2
	if (pREQ->PrimaryLocalAckTimeout >= 1)
	{
		pConnectRequest->PathInfo.Path.PktLifeTime = (pREQ->PrimaryLocalAckTimeout-1);
	} else {
		pConnectRequest->PathInfo.Path.PktLifeTime = 0;
	}


	pConnectRequest->PathInfo.Path.PktLifeTimeSelector	= IB_SELECTOR_EQ;

	// Alternate path info
	if (pREQ->AlternateLocalLID)
	{
		pConnectRequest->AlternatePathInfo.bSubnetLocal	= (boolean)pREQ->AlternateSubnetLocal;

		pConnectRequest->AlternatePathInfo.Path.SLID	= pREQ->AlternateRemoteLID;
		pConnectRequest->AlternatePathInfo.Path.SGID	= pREQ->AlternateRemoteGID;

		pConnectRequest->AlternatePathInfo.Path.DLID	= pREQ->AlternateLocalLID;
		pConnectRequest->AlternatePathInfo.Path.DGID	= pREQ->AlternateLocalGID;


		pConnectRequest->AlternatePathInfo.Path.NumbPath= 1; // Only one path.
		pConnectRequest->AlternatePathInfo.Path.u1.s.FlowLabel	= pREQ->u6.s.AlternateFlowLabel;
		pConnectRequest->AlternatePathInfo.Path.u1.s.HopLimit	= pREQ->AlternateHopLimit;
		pConnectRequest->AlternatePathInfo.Path.TClass	= (uint8)pREQ->AlternateTrafficClass;
		pConnectRequest->AlternatePathInfo.Path.u2.s.SL	= (uint8)pREQ->AlternateSL;
		pConnectRequest->AlternatePathInfo.Path.Rate	= (uint8)pREQ->u6.s.AlternatePacketRate;
		pConnectRequest->AlternatePathInfo.Path.RateSelector= IB_SELECTOR_EQ;

		pConnectRequest->AlternatePathInfo.Path.P_Key	= pREQ->PartitionKey;
		pConnectRequest->AlternatePathInfo.Path.Mtu		= (uint8)pREQ->PathMTU;
	
		pConnectRequest->AlternatePathInfo.Path.MtuSelector	= IB_SELECTOR_EQ;

		// since exponent, subtracting 1 means divide by 2
		// this value ends up rounded up by Ca Ack Delay/2
		if (pREQ->AlternateLocalAckTimeout >= 1)
		{
			pConnectRequest->AlternatePathInfo.Path.PktLifeTime = (pREQ->AlternateLocalAckTimeout-1);
		} else {
			pConnectRequest->AlternatePathInfo.Path.PktLifeTime = 0;
		}
	}
	else
	{
		pConnectRequest->AlternatePathInfo.bSubnetLocal	= 0;

		pConnectRequest->AlternatePathInfo.Path.SLID	= 0;
		MemoryClear(&pConnectRequest->AlternatePathInfo.Path.SGID.Raw, sizeof(IB_GID));

		pConnectRequest->AlternatePathInfo.Path.DLID	= 0;
		MemoryClear(&pConnectRequest->AlternatePathInfo.Path.DGID.Raw, sizeof(IB_GID));

		pConnectRequest->AlternatePathInfo.Path.NumbPath	= 0;
		pConnectRequest->AlternatePathInfo.Path.u1.s.FlowLabel	= 0;
		pConnectRequest->AlternatePathInfo.Path.u1.s.HopLimit	= 0;
		pConnectRequest->AlternatePathInfo.Path.TClass		= 0;
		pConnectRequest->AlternatePathInfo.Path.u2.s.SL		= 0;
		pConnectRequest->AlternatePathInfo.Path.Rate		= 0;
		pConnectRequest->AlternatePathInfo.Path.RateSelector= 0;

		pConnectRequest->AlternatePathInfo.Path.P_Key		= 0;
		pConnectRequest->AlternatePathInfo.Path.Mtu			= 0;
		pConnectRequest->AlternatePathInfo.Path.MtuSelector	= 0;
		pConnectRequest->AlternatePathInfo.Path.PktLifeTime	= 0;
	}

	pConnectRequest->SID								= pREQ->ServiceID;
	pConnectRequest->CEPInfo.CaGUID						= pREQ->LocalCaGUID;
	pConnectRequest->CEPInfo.EndToEndFlowControl		= (uint8)pREQ->u3.s.EndToEndFlowControl;
	//pConnectRequest->CEPInfo.PortGUID					= handled by caller

	// QP info
	pConnectRequest->CEPInfo.StartingPSN				= pREQ->u4.s.StartingPSN;
	pConnectRequest->CEPInfo.QPN						= pREQ->u1.s.LocalQPN;
	pConnectRequest->CEPInfo.QKey						= pREQ->LocalQKey;
	pConnectRequest->CEPInfo.RetryCount					= (uint8)pREQ->u4.s.RetryCount;
	pConnectRequest->CEPInfo.RnrRetryCount				= (uint8)pREQ->RnRRetryCount;
	pConnectRequest->CEPInfo.AckTimeout					= (uint8)pREQ->PrimaryLocalAckTimeout;
	pConnectRequest->CEPInfo.AlternateAckTimeout		= (uint8)pREQ->AlternateLocalAckTimeout;
	pConnectRequest->CEPInfo.OfferedResponderResources	= (uint8)pREQ->u1.s.OfferedResponderResources;
	pConnectRequest->CEPInfo.OfferedInitiatorDepth		= (uint8)pREQ->u2.s.OfferedInitiatorDepth;

	pConnectRequest->CEPInfo.LocalEECN					= pREQ->u2.s.LocalEECN;
	pConnectRequest->CEPInfo.RemoteEECN					= pREQ->u3.s.RemoteEECN;

	memcpy(pConnectRequest->PrivateData, pREQ->PrivateData, CMM_REQ_USER_LEN);

	DUMP_REQ(pREQ);

} // CopyREQToReqInfo()


//////////////////////////////////////////////////////////////////////////
// FormatREP
//
// Format the REP message
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	None.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//

void
FormatREP(CM_MAD* Mad, 
		  const CM_REPLY_INFO* pConnectReply,
		  uint64 TransactionID,
		  uint32 LocalCommID,
		  uint32 RemoteCommID,
		  uint64 LocalCaGUID
		  )
{
	// Set up the mad header
	MAD_SET_VERSION_INFO(Mad, IB_BASE_VERSION, MCLASS_COMM_MGT, IB_COMM_MGT_CLASS_VERSION);
	
	MAD_SET_ATTRIB_ID(Mad, MCLASS_ATTRIB_ID_REP); // Reply msg

	MAD_SET_METHOD_TYPE(Mad, MMTHD_SEND);

	MAD_SET_TRANSACTION_ID(Mad, TransactionID);

	// Set up the REP msg
	Mad->payload.REP.LocalCommID			= LocalCommID;
	Mad->payload.REP.RemoteCommID			= RemoteCommID;
	Mad->payload.REP.LocalQKey				= pConnectReply->QKey;
	Mad->payload.REP.u1.s.LocalQPN			= pConnectReply->QPN;
	Mad->payload.REP.u1.s.Reserved1			= 0;
	Mad->payload.REP.u2.s.LocalEECN			= pConnectReply->EECN;
	Mad->payload.REP.u2.s.Reserved2			= 0;
	Mad->payload.REP.u3.s.StartingPSN		= pConnectReply->StartingPSN;
	Mad->payload.REP.u3.s.Reserved3			= 0;
	Mad->payload.REP.ArbResponderResources	= pConnectReply->ArbResponderResources;
	Mad->payload.REP.ArbInitiatorDepth		= pConnectReply->ArbInitiatorDepth;
	Mad->payload.REP.TargetAckDelay			= pConnectReply->TargetAckDelay;
	Mad->payload.REP.FailoverAccepted		= pConnectReply->FailoverAccepted;
	Mad->payload.REP.EndToEndFlowControl	= pConnectReply->EndToEndFlowControl;
	Mad->payload.REP.RnRRetryCount			= pConnectReply->RnRRetryCount;
	Mad->payload.REP.Reserved4				= 0;

	Mad->payload.REP.LocalCaGUID			= LocalCaGUID;

	memcpy(Mad->payload.REP.PrivateData, pConnectReply->PrivateData, CMM_REP_USER_LEN);

	DUMP_REP(&Mad->payload.REP);

	ConvertMadToNetworkByteOrder(Mad);

} // FormatREP()

void
CopyREPToRepInfo( 
	const CMM_REP *pREP,
	CM_REPLY_INFO *pReplyInfo)
{
	pReplyInfo->QKey = pREP->LocalQKey;

	pReplyInfo->QPN = pREP->u1.s.LocalQPN;

	pReplyInfo->EECN = pREP->u2.s.LocalEECN;

	pReplyInfo->StartingPSN = pREP->u3.s.StartingPSN;

	pReplyInfo->ArbResponderResources = pREP->ArbResponderResources;
	pReplyInfo->ArbInitiatorDepth = pREP->ArbInitiatorDepth;
	pReplyInfo->TargetAckDelay = pREP->TargetAckDelay;
	pReplyInfo->FailoverAccepted = pREP->FailoverAccepted;
	pReplyInfo->EndToEndFlowControl = pREP->EndToEndFlowControl;
	pReplyInfo->RnRRetryCount = pREP->RnRRetryCount;
	pReplyInfo->CaGUID = pREP->LocalCaGUID;

	MemoryCopy( pReplyInfo->PrivateData, pREP->PrivateData, CMM_REP_USER_LEN );

	DUMP_REP(pREP);

} // CopyREPToRepInfo()

void
FormatREJ(CM_MAD*	Mad,
		  uint8		MsgRejected,	//	CMM_REJ_MSGTYPE
		  uint16	Reason,
		  const uint8*	pRejectInfo,
		  uint32	RejectInfoLen,
		  const uint8*	pPrivateData,
		  uint32	PrivateDataLen,
		  uint64	TransactionID,
		  uint32	LocalCommID,
		  uint32	RemoteCommID)
{
	// Set up the mad header
	MAD_SET_VERSION_INFO(Mad, IB_BASE_VERSION, MCLASS_COMM_MGT, IB_COMM_MGT_CLASS_VERSION);
	
	MAD_SET_ATTRIB_ID(Mad, MCLASS_ATTRIB_ID_REJ); // reject msg

	MAD_SET_METHOD_TYPE(Mad, MMTHD_SEND);

	MAD_SET_TRANSACTION_ID(Mad, TransactionID);

	// Set up the REJ msg
	Mad->payload.REJ.LocalCommID			= LocalCommID;
	Mad->payload.REJ.RemoteCommID			= RemoteCommID;

	Mad->payload.REJ.MsgRejected			= MsgRejected;
	Mad->payload.REJ.Reserved1				= 0;
	if (RejectInfoLen > CMM_REJ_ADD_INFO_LEN)
		RejectInfoLen = CMM_REJ_ADD_INFO_LEN;
	Mad->payload.REJ.RejectInfoLen			= (uint8)RejectInfoLen;
	Mad->payload.REJ.Reserved2				= 0;
	Mad->payload.REJ.Reason					= Reason;
	
	if (RejectInfoLen)
		memcpy(Mad->payload.REJ.RejectInfo, pRejectInfo, RejectInfoLen);

	memset(&Mad->payload.REJ.RejectInfo[RejectInfoLen], 0, CMM_REJ_ADD_INFO_LEN - RejectInfoLen);

	if (PrivateDataLen)
		memcpy(Mad->payload.REJ.PrivateData, pPrivateData, PrivateDataLen);

	memset(&Mad->payload.REJ.PrivateData[PrivateDataLen], 0, CMM_REJ_USER_LEN - PrivateDataLen);

	DUMP_REJ(&Mad->payload.REJ);

	ConvertMadToNetworkByteOrder(Mad);

} // FormatREJ()

void
CopyREJToRejectInfo(
	const CMM_REJ* pREJ,
	CM_REJECT_INFO *pRejectInfo)
{
	pRejectInfo->RejectInfoLen = MIN(pREJ->RejectInfoLen, CMM_REJ_USER_LEN);

	pRejectInfo->Reason = pREJ->Reason;

	memcpy(pRejectInfo->RejectInfo, pREJ->RejectInfo, pRejectInfo->RejectInfoLen);

	memcpy(pRejectInfo->PrivateData, pREJ->PrivateData, CMM_REJ_USER_LEN);

	DUMP_REJ(pREJ);

} // CopyREJToRejectInfo

void
FormatMRA(CM_MAD* Mad, 
		  uint8	MsgMraed,			// CMM_MRA_MSGTYPE
		  uint8	ServiceTimeout,
		  const uint8* pPrivateData,
		  uint32 PrivateDataLen,
		  uint64 TransactionID,
		  uint32 LocalCommID,
		  uint32 RemoteCommID)
{
	// Set up the mad header
	MAD_SET_VERSION_INFO(Mad, IB_BASE_VERSION, MCLASS_COMM_MGT, IB_COMM_MGT_CLASS_VERSION);
	
	MAD_SET_ATTRIB_ID(Mad, MCLASS_ATTRIB_ID_MRA);

	MAD_SET_METHOD_TYPE(Mad, MMTHD_SEND);

	MAD_SET_TRANSACTION_ID(Mad, TransactionID);

	// Set up the MRA msg
	Mad->payload.MRA.LocalCommID	= LocalCommID;
	Mad->payload.MRA.RemoteCommID	= RemoteCommID;

	Mad->payload.MRA.MsgMraed		= MsgMraed;
	Mad->payload.MRA.Reserved1		= 0;
	Mad->payload.MRA.ServiceTimeout = ServiceTimeout;
	Mad->payload.MRA.Reserved2		= 0;

	if (PrivateDataLen)
		memcpy(Mad->payload.MRA.PrivateData, pPrivateData, PrivateDataLen);

	memset(Mad->payload.MRA.PrivateData + PrivateDataLen, 0, CMM_MRA_USER_LEN - PrivateDataLen);

	ConvertMadToNetworkByteOrder(Mad);

} // FormatMRA()


void
FormatRTU(CM_MAD* Mad, 
		  const uint8* pPrivateData,
		  uint32 PrivateDataLen,
		  uint64 TransactionID,
		  uint32 LocalCommID,
		  uint32 RemoteCommID)
{
	// Set up the mad header
	MAD_SET_VERSION_INFO(Mad, IB_BASE_VERSION, MCLASS_COMM_MGT, IB_COMM_MGT_CLASS_VERSION);
	
	MAD_SET_ATTRIB_ID(Mad, MCLASS_ATTRIB_ID_RTU); // Ready-to-use msg

	MAD_SET_METHOD_TYPE(Mad, MMTHD_SEND);

	MAD_SET_TRANSACTION_ID(Mad, TransactionID);

	Mad->payload.RTU.LocalCommID	= LocalCommID;
	Mad->payload.RTU.RemoteCommID	= RemoteCommID;

	if (PrivateDataLen)
		memcpy(Mad->payload.RTU.PrivateData, pPrivateData, PrivateDataLen);

	memset(Mad->payload.RTU.PrivateData + PrivateDataLen, 0, CMM_RTU_USER_LEN - PrivateDataLen);

	ConvertMadToNetworkByteOrder(Mad);

} // FormatRTU()

void
FormatLAP(
		  IN CM_MAD* Mad, 
		  IN uint32 QPN,
		  IN uint32 RemoteCMTimeout,
		  IN const CM_ALTPATH_INFO*	pAltPath,
		  IN uint8  AlternateAckTimeout,
		  IN uint64 TransactionID,
		  IN uint32 LocalCommID,
		  IN uint32 RemoteCommID
		  )
{
	// Set up the mad header
	MAD_SET_VERSION_INFO(Mad, IB_BASE_VERSION, MCLASS_COMM_MGT, IB_COMM_MGT_CLASS_VERSION);
	
	MAD_SET_ATTRIB_ID(Mad, MCLASS_ATTRIB_ID_LAP); // Load Alternate Path

	MAD_SET_METHOD_TYPE(Mad, MMTHD_SEND);

	MAD_SET_TRANSACTION_ID(Mad, TransactionID);

	Mad->payload.LAP.LocalCommID	= LocalCommID;
	Mad->payload.LAP.RemoteCommID	= RemoteCommID;
	Mad->payload.LAP.Reserved1		= 0;
	Mad->payload.LAP.u1.s.RemoteQPNorEECN = QPN;
	Mad->payload.LAP.u1.s.RemoteCMTimeout = RemoteCMTimeout;
	Mad->payload.LAP.u1.s.Reserved2	= 0;
	Mad->payload.LAP.Reserved3		= 0;

	Mad->payload.LAP.AlternateLocalLID			= pAltPath->AlternatePathInfo.Path.SLID;
	Mad->payload.LAP.AlternateRemoteLID			= pAltPath->AlternatePathInfo.Path.DLID;
	Mad->payload.LAP.AlternateLocalGID			= pAltPath->AlternatePathInfo.Path.SGID;
	Mad->payload.LAP.AlternateRemoteGID			= pAltPath->AlternatePathInfo.Path.DGID;

	Mad->payload.LAP.u2.s.AlternateFlowLabel	= pAltPath->AlternatePathInfo.Path.u1.s.FlowLabel;
	Mad->payload.LAP.u2.s.Reserved4				= 0;
	Mad->payload.LAP.u2.s.AlternateTrafficClass		= pAltPath->AlternatePathInfo.Path.TClass;
	Mad->payload.LAP.AlternateHopLimit			= pAltPath->AlternatePathInfo.Path.u1.s.HopLimit;
	Mad->payload.LAP.Reserved5					= 0;
	Mad->payload.LAP.AlternatePacketRate		= pAltPath->AlternatePathInfo.Path.Rate;
	Mad->payload.LAP.AlternateSL				= pAltPath->AlternatePathInfo.Path.u2.s.SL;

	Mad->payload.LAP.AlternateSubnetLocal		= pAltPath->AlternatePathInfo.bSubnetLocal;
	Mad->payload.LAP.Reserved6					= 0;
	Mad->payload.LAP.AlternateLocalAckTimeout	= AlternateAckTimeout;
	Mad->payload.LAP.Reserved7					= 0;

	memcpy(Mad->payload.LAP.PrivateData, pAltPath->PrivateData, CMM_LAP_USER_LEN);

	ConvertMadToNetworkByteOrder(Mad);

	DUMP_LAP(&Mad->payload.LAP);
}

void
CopyLAPToAltPathRequest(
	IN const CMM_LAP* pLAP,
	IN OUT CM_ALTPATH_INFO *pAltPath
	)
{
	pAltPath->AlternateAckTimeout=	pLAP->AlternateLocalAckTimeout;

	pAltPath->AlternatePathInfo.Path.SLID=	pLAP->AlternateLocalLID;
	pAltPath->AlternatePathInfo.Path.DLID=	pLAP->AlternateRemoteLID;
	pAltPath->AlternatePathInfo.Path.SGID=	pLAP->AlternateLocalGID;
	pAltPath->AlternatePathInfo.Path.DGID=	pLAP->AlternateRemoteGID;
	pAltPath->AlternatePathInfo.Path.NumbPath			= 1; // Only one path.

	pAltPath->AlternatePathInfo.Path.u1.s.FlowLabel=pLAP->u2.s.AlternateFlowLabel;
	pAltPath->AlternatePathInfo.Path.TClass=pLAP->u2.s.AlternateTrafficClass;
	pAltPath->AlternatePathInfo.Path.u1.s.HopLimit=	pLAP->AlternateHopLimit;
	pAltPath->AlternatePathInfo.Path.Rate=	pLAP->AlternatePacketRate;
	pAltPath->AlternatePathInfo.Path.RateSelector	= IB_SELECTOR_EQ;
	pAltPath->AlternatePathInfo.Path.u2.s.SL=	pLAP->AlternateSL;
	//pAltPath->AlternatePathInfo.Path.P_Key	= caller handles
	//pAltPath->AlternatePathInfo.Path.Mtu	= caller handles
	pAltPath->AlternatePathInfo.Path.MtuSelector	= IB_SELECTOR_EQ;

	pAltPath->AlternatePathInfo.bSubnetLocal=	pLAP->AlternateSubnetLocal;

	// since exponent, subtracting 1 means divide by 2
	// this value ends up rounded up by Ca Ack Delay/2
	if (pLAP->AlternateLocalAckTimeout >= 1)
	{
		pAltPath->AlternatePathInfo.Path.PktLifeTime = (pLAP->AlternateLocalAckTimeout-1);
	} else {
		pAltPath->AlternatePathInfo.Path.PktLifeTime = 0;
	}
	pAltPath->AlternatePathInfo.Path.PktLifeTimeSelector	= IB_SELECTOR_EQ;

	memcpy(pAltPath->PrivateData, pLAP->PrivateData, CMM_LAP_USER_LEN);

	DUMP_LAP(pLAP);
}

// performs copy of a received LAP to given AltPath
void
CopyRcvdLAPToAltPath(
	IN const CMM_LAP* pLAP,
	OUT CM_CEP_PATH *pAltPath
	)
{
	// caller handles pAltPath->LocalPortGuid
	pAltPath->LocalGID =	pLAP->AlternateRemoteGID;
	pAltPath->RemoteGID =	pLAP->AlternateLocalGID;
	pAltPath->LocalLID =	pLAP->AlternateRemoteLID;
	pAltPath->RemoteLID =	pLAP->AlternateLocalLID;

	pAltPath->FlowLabel=pLAP->u2.s.AlternateFlowLabel;
	pAltPath->StaticRate=	pLAP->AlternatePacketRate;
	// caller handles pAltPath->PkeyIndex
	pAltPath->TClass=pLAP->u2.s.AlternateTrafficClass;
	pAltPath->SL=	pLAP->AlternateSL;
	pAltPath->bSubnetLocal=	pLAP->AlternateSubnetLocal;
	pAltPath->AckTimeout=	pLAP->AlternateLocalAckTimeout;
	pAltPath->LocalAckTimeout=	pLAP->AlternateLocalAckTimeout;
	pAltPath->Reserved = 0;
	pAltPath->HopLimit=	pLAP->AlternateHopLimit;
	// caller handles pAltPath->LocalGidIndex
}

// performs copy of a sent LAP to given AltPath
void
CopySentLAPToAltPath(
	IN const CMM_LAP* pLAP,
	OUT CM_CEP_PATH *pAltPath
	)
{
	// caller handles pAltPath->LocalPortGuid
	pAltPath->LocalGID =	pLAP->AlternateLocalGID;
	pAltPath->RemoteGID =	pLAP->AlternateRemoteGID;
	pAltPath->LocalLID =	pLAP->AlternateLocalLID;
	pAltPath->RemoteLID =	pLAP->AlternateRemoteLID;

	pAltPath->FlowLabel=pLAP->u2.s.AlternateFlowLabel;
	pAltPath->StaticRate=	pLAP->AlternatePacketRate;
	// caller handles pAltPath->PkeyIndex
	pAltPath->TClass=pLAP->u2.s.AlternateTrafficClass;
	pAltPath->SL=	pLAP->AlternateSL;
	pAltPath->bSubnetLocal=	pLAP->AlternateSubnetLocal;
	pAltPath->AckTimeout=	pLAP->AlternateLocalAckTimeout;
	// caller handles pAltPath->LocalAckTimeout
	pAltPath->Reserved = 0;
	pAltPath->HopLimit=	pLAP->AlternateHopLimit;
	// caller handles pAltPath->LocalGidIndex
}

// compares received LAP to given Path
// return 0 on match
int
CompareRcvdLAPToPath(
	IN const CMM_LAP* pLAP,
	IN const CM_CEP_PATH *pPath
	)
{
	if (pPath->LocalLID != pLAP->AlternateRemoteLID)
		return 1;
	if (pPath->RemoteLID !=	pLAP->AlternateLocalLID)
		return 1;
	// we must test GIDs, if ports are on different fabrics, the LIDs
	// could match but the GIDs would be different
	if (0 != MemoryCompare(&pPath->LocalGID, &pLAP->AlternateRemoteGID, sizeof(IB_GID)))
		return 1;
	if (0 != MemoryCompare(&pPath->RemoteGID, &pLAP->AlternateLocalGID, sizeof(IB_GID)))
		return 1;
	if (pPath->bSubnetLocal != pLAP->AlternateSubnetLocal)
		return 1;

	// ignore minor parameters
	//pPath->FlowLabel=pLAP->u2.s.AlternateFlowLabel;
	//pPath->StaticRate=	pLAP->AlternatePacketRate;
	//pPath->TClass=pLAP->u2.s.AlternateTrafficClass;
	//pPath->SL=	pLAP->AlternateSL;
	//pPath->AckTimeout=	pLAP->AlternateLocalAckTimeout;
	//pPath->HopLimit=	pLAP->AlternateHopLimit;
	return 0;
}

// compares two LAPs
// return 0 on match
int
CompareLAP(
	IN const CMM_LAP* pLAP1,
	IN const CMM_LAP* pLAP2
	)
{
	if (pLAP1->AlternateLocalAckTimeout != pLAP2->AlternateLocalAckTimeout)
		return 1;

	if (pLAP1->AlternateLocalLID != pLAP2->AlternateLocalLID)
		return 1;
	if (pLAP1->AlternateRemoteLID != pLAP2->AlternateRemoteLID)
		return 1;
	if (0 != MemoryCompare(&pLAP1->AlternateLocalGID, &pLAP2->AlternateLocalGID, sizeof(IB_GID)))
		return 1;
	if (0 != MemoryCompare(&pLAP1->AlternateRemoteGID, &pLAP2->AlternateRemoteGID, sizeof(IB_GID)))
		return 1;
	if (pLAP1->u2.s.AlternateFlowLabel != pLAP2->u2.s.AlternateFlowLabel)
		return 1;
	if (pLAP1->u2.s.AlternateTrafficClass != pLAP2->u2.s.AlternateTrafficClass)
		return 1;
	if (pLAP1->AlternateHopLimit != pLAP2->AlternateHopLimit)
		return 1;
	if (pLAP1->AlternatePacketRate != pLAP2->AlternatePacketRate)
		return 1;
	if (pLAP1->AlternateSL != pLAP2->AlternateSL)
		return 1;
	if (pLAP1->AlternateSubnetLocal != pLAP2->AlternateSubnetLocal)
		return 1;
	if (0 != MemoryCompare(pLAP1->PrivateData, pLAP2->PrivateData, CMM_LAP_USER_LEN))
		return 1;
	return 0;
}
void
FormatAPR(
		  IN CM_MAD*	Mad,
		  IN CM_APR_STATUS	APStatus,
		  IN const uint8*	pAddInfo,
		  IN uint32		AddInfoLen,
		  IN const uint8*	pPrivateData,
		  IN uint32		PrivateDataLen,
		  IN uint64 	TransactionID,
		  IN uint32		LocalCommID,
		  IN uint32		RemoteCommID
		  )
{
	// Set up the mad header
	MAD_SET_VERSION_INFO(Mad, IB_BASE_VERSION, MCLASS_COMM_MGT, IB_COMM_MGT_CLASS_VERSION);
	
	MAD_SET_ATTRIB_ID(Mad, MCLASS_ATTRIB_ID_APR); // Alternate Path Reply

	MAD_SET_METHOD_TYPE(Mad, MMTHD_SEND);

	MAD_SET_TRANSACTION_ID(Mad, TransactionID);

	// Set up the APR msg
	Mad->payload.APR.LocalCommID			= LocalCommID;
	Mad->payload.APR.RemoteCommID			= RemoteCommID;

	Mad->payload.APR.APStatus				= APStatus;
	if (AddInfoLen > CMM_APR_ADD_INFO_LEN)
		AddInfoLen = CMM_APR_ADD_INFO_LEN;
	Mad->payload.APR.AddInfoLen				= (uint8)AddInfoLen;
	Mad->payload.APR.Reserved				= 0;
	
	if (AddInfoLen)
		memcpy(Mad->payload.APR.AddInfo, pAddInfo, AddInfoLen);

	memset(&Mad->payload.APR.AddInfo[AddInfoLen], 0, CMM_APR_ADD_INFO_LEN - AddInfoLen);

	if (PrivateDataLen)
		memcpy(Mad->payload.APR.PrivateData, pPrivateData, PrivateDataLen);

	memset(&Mad->payload.APR.PrivateData[PrivateDataLen], 0, CMM_APR_USER_LEN - PrivateDataLen);

	DUMP_APR(&Mad->payload.APR);

	ConvertMadToNetworkByteOrder(Mad);
}

void
CopyAPRToAltPathReply(
	IN const CMM_APR* pAPR,
	IN OUT CM_ALTPATH_REPLY_INFO *pAltPathReply
	)
{
	pAltPathReply->APStatus = pAPR->APStatus;
	// for APS_PATH_LOADED, caller will populate PathInfo and AckTimeout
	if (pAPR->APStatus != APS_PATH_LOADED)
	{
		pAltPathReply->u.AddInfo.Len = MIN(pAPR->AddInfoLen,CMM_APR_ADD_INFO_LEN);
		pAltPathReply->u.AddInfo.Reserved = 0;

		memcpy(pAltPathReply->u.AddInfo.Info, pAPR->AddInfo, pAltPathReply->u.AddInfo.Len);
	}
	memcpy(pAltPathReply->PrivateData, pAPR->PrivateData, CMM_APR_USER_LEN);

	DUMP_APR(pAPR);
}

void
FormatDREQ(CM_MAD*	Mad,
		   uint32	RemoteQPNorEECN,
		   const uint8*	pPrivateData,
		   uint32	PrivateDataLen,
		   uint64 	TransactionID,
		   uint32	LocalCommID,
		   uint32	RemoteCommID)
{
	// Set up the mad header
	MAD_SET_VERSION_INFO(Mad, IB_BASE_VERSION, MCLASS_COMM_MGT, IB_COMM_MGT_CLASS_VERSION);
	
	MAD_SET_ATTRIB_ID(Mad, MCLASS_ATTRIB_ID_DREQ); // Disconnect request msg

	MAD_SET_METHOD_TYPE(Mad, MMTHD_SEND);

	MAD_SET_TRANSACTION_ID(Mad, TransactionID);

	// Set up the DREQ msg
	Mad->payload.DREQ.LocalCommID			= LocalCommID;
	Mad->payload.DREQ.RemoteCommID			= RemoteCommID;

	Mad->payload.DREQ.u.s.RemoteQPNorEECN	= RemoteQPNorEECN;
	Mad->payload.DREQ.u.s.Reserved			= 0;

	if (PrivateDataLen)
		memcpy(Mad->payload.DREQ.PrivateData, pPrivateData, PrivateDataLen);

	memset(Mad->payload.DREQ.PrivateData + PrivateDataLen, 0, CMM_DREQ_USER_LEN - PrivateDataLen);

	DUMP_DREQ(&Mad->payload.DREQ);

	ConvertMadToNetworkByteOrder(Mad);

} // FormatDREQ()


void
FormatDREP(CM_MAD*	Mad,
			const uint8*	pPrivateData,
			uint32	PrivateDataLen,
		   	uint64 	TransactionID,
			uint32 LocalCommID,
			uint32 RemoteCommID)

{
	// Set up the mad header
	MAD_SET_VERSION_INFO(Mad, IB_BASE_VERSION, MCLASS_COMM_MGT, IB_COMM_MGT_CLASS_VERSION);
	
	MAD_SET_ATTRIB_ID(Mad, MCLASS_ATTRIB_ID_DREP); // Disconnect reply msg

	MAD_SET_METHOD_TYPE(Mad, MMTHD_SEND);

	MAD_SET_TRANSACTION_ID(Mad, TransactionID);

	// Set up the DREP msg
	Mad->payload.DREP.LocalCommID			= LocalCommID;
	Mad->payload.DREP.RemoteCommID			= RemoteCommID;

	if (PrivateDataLen)
		memcpy(Mad->payload.DREP.PrivateData, pPrivateData, PrivateDataLen);

	memset(Mad->payload.DREP.PrivateData + PrivateDataLen, 0, CMM_DREP_USER_LEN - PrivateDataLen);

	ConvertMadToNetworkByteOrder(Mad);

} // FormatDREP()


void
FormatSIDR_REQ(CM_MAD* Mad,
				uint64 SID,
				uint16 PartitionKey,
				const uint8* pPrivateData,
				uint32 PrivateDataLen,
				uint64 TransactionID,
				uint32 RequestID)
{
	// Set up the mad header
	MAD_SET_VERSION_INFO(Mad, IB_BASE_VERSION, MCLASS_COMM_MGT, IB_COMM_MGT_CLASS_VERSION);
	
	MAD_SET_ATTRIB_ID(Mad, MCLASS_ATTRIB_ID_SIDR_REQ); // SIDR request

	MAD_SET_METHOD_TYPE(Mad, MMTHD_SEND);

	MAD_SET_TRANSACTION_ID(Mad, TransactionID);

	// Set up the SIDR REQ
	Mad->payload.SIDR_REQ.RequestID	= RequestID;
	Mad->payload.SIDR_REQ.PartitionKey= PartitionKey;
	Mad->payload.SIDR_REQ.Reserved	= 0;
	Mad->payload.SIDR_REQ.ServiceID	= SID;

	if (PrivateDataLen)
		memcpy(Mad->payload.SIDR_REQ.PrivateData, pPrivateData, PrivateDataLen);

	memset(Mad->payload.SIDR_REQ.PrivateData + PrivateDataLen, 0, CMM_SIDR_REQ_USER_LEN - PrivateDataLen);

	DUMP_SIDR_REQ(&Mad->payload.SIDR_REQ);

	ConvertMadToNetworkByteOrder(Mad);

} // FormatSIDR_REQ()

void
CopySIDR_REQToSidrReqInfo(
	const CMM_SIDR_REQ* pSIDR_REQ,
	SIDR_REQ_INFO *pSidrReqInfo)
{
	pSidrReqInfo->SID = pSIDR_REQ->ServiceID;
	pSidrReqInfo->PartitionKey = pSIDR_REQ->PartitionKey;
	pSidrReqInfo->PathInfo.Path.P_Key = pSIDR_REQ->PartitionKey;

	memcpy(pSidrReqInfo->PrivateData, pSIDR_REQ->PrivateData, CMM_SIDR_REQ_USER_LEN);

	DUMP_SIDR_REQ(pSIDR_REQ);

} // CopySIDR_REQToSidrReqInfo()

void
FormatSIDR_RESP(CM_MAD* Mad,
				uint64 SID,
				uint32 QPN,
				uint32 QKey,
				uint32 Status,
				const uint8*	pPrivateData,
				uint32	PrivateDataLen,
				uint64 TransactionID,
				uint32 RequestID)
{
	// Set up the mad header
	MAD_SET_VERSION_INFO(Mad, IB_BASE_VERSION, MCLASS_COMM_MGT, IB_COMM_MGT_CLASS_VERSION);
	
	MAD_SET_ATTRIB_ID(Mad, MCLASS_ATTRIB_ID_SIDR_RESP); // SIDR response

	MAD_SET_METHOD_TYPE(Mad, MMTHD_SEND);

	MAD_SET_TRANSACTION_ID(Mad, TransactionID);

	// Set up the SIDR REP
	Mad->payload.SIDR_RESP.RequestID	= RequestID;
	Mad->payload.SIDR_RESP.Status		= Status;
	Mad->payload.SIDR_RESP.AddInfoLen	= 0;
	Mad->payload.SIDR_RESP.Reserved		= 0;
	Mad->payload.SIDR_RESP.u.s.QPN		= QPN;
	Mad->payload.SIDR_RESP.u.s.Reserved2= 0;
	Mad->payload.SIDR_RESP.ServiceID	= SID;
	Mad->payload.SIDR_RESP.QKey			= QKey;

	// TODO: If we support redir, need to add the port info here
	memset(Mad->payload.SIDR_RESP.AddInfo, 0, CMM_SIDR_RESP_ADD_INFO_LEN);

	if (PrivateDataLen)
		memcpy(Mad->payload.SIDR_RESP.PrivateData, pPrivateData, PrivateDataLen);

	memset(Mad->payload.SIDR_RESP.PrivateData + PrivateDataLen, 0, CMM_SIDR_RESP_USER_LEN - PrivateDataLen);

	DUMP_SIDR_RESP(&Mad->payload.SIDR_RESP);

	ConvertMadToNetworkByteOrder(Mad);

} // FormatSIDR_RESP()

void
CopySIDR_RESPToSidrRespInfo(
	const CMM_SIDR_RESP* pSIDR_RESP,
	SIDR_RESP_INFO *pSidrRespInfo)
{
	pSidrRespInfo->QPN = pSIDR_RESP->u.s.QPN;

	pSidrRespInfo->QKey = pSIDR_RESP->QKey;

	pSidrRespInfo->Status = pSIDR_RESP->Status;

	memcpy(pSidrRespInfo->PrivateData, pSIDR_RESP->PrivateData, CMM_SIDR_RESP_USER_LEN);

	DUMP_SIDR_RESP(pSIDR_RESP);

} // CopySIDR_RESPToSidrRespInfo()


FSTATUS
CopyMadToConnInfo(
	CONN_INFO*		pConnInfo,
	const CM_MAD*			pMad)
{
	FSTATUS Status=FSUCCESS;
	uint32	AttrID=0;


	MAD_GET_ATTRIB_ID(pMad, &AttrID);

	switch (AttrID)
	{
	case MCLASS_ATTRIB_ID_REQ:
		CopyREQToReqInfo(&pMad->payload.REQ, &pConnInfo->Request);
		break;

	case MCLASS_ATTRIB_ID_REP:
		CopyREPToRepInfo(&pMad->payload.REP, &pConnInfo->Reply );
		break;

	case MCLASS_ATTRIB_ID_RTU:
		// Copy the private data only
		memcpy(pConnInfo->Rtu.PrivateData, pMad->payload.RTU.PrivateData, CM_RTU_INFO_USER_LEN);
		break;

	case MCLASS_ATTRIB_ID_REJ:
		CopyREJToRejectInfo(&pMad->payload.REJ, &pConnInfo->Reject);
		break;

	case MCLASS_ATTRIB_ID_DREQ:
		// Copy the private data only
		memcpy(pConnInfo->DisconnectRequest.PrivateData, pMad->payload.DREQ.PrivateData, CM_DREQUEST_INFO_USER_LEN);
		break;

	case MCLASS_ATTRIB_ID_DREP:
		// Copy the private data only
		memcpy(pConnInfo->DisconnectReply.PrivateData, pMad->payload.DREP.PrivateData, CM_DREPLY_INFO_USER_LEN);
		break;

	case MCLASS_ATTRIB_ID_SIDR_REQ:
		CopySIDR_REQToSidrReqInfo(&pMad->payload.SIDR_REQ, &pConnInfo->SidrRequest);
		break;

	case MCLASS_ATTRIB_ID_SIDR_RESP:
		CopySIDR_RESPToSidrRespInfo(&pMad->payload.SIDR_RESP, &pConnInfo->SidrResponse);
		break;

	case MCLASS_ATTRIB_ID_MRA:
		// no action needed, internally handled in CM
		break;

	case MCLASS_ATTRIB_ID_LAP:
		CopyLAPToAltPathRequest(&pMad->payload.LAP, &pConnInfo->AltPathRequest);
		break;

	case MCLASS_ATTRIB_ID_APR:
		CopyAPRToAltPathReply(&pMad->payload.APR, &pConnInfo->AltPathReply);
		break;

	default:
		Status = FCM_INVALID_EVENT;
		break;

	} // switch()

	return Status;

} // CopyMadToConnInfo()



void
ConvertREQToNetworkOrder(CMM_REQ* pRequest)
{
	// Set up the REQ msg
	pRequest->LocalCommID				= hton32(pRequest->LocalCommID);
	pRequest->Reserved1					= 0;
	pRequest->ServiceID					= hton64(pRequest->ServiceID);
	pRequest->LocalCaGUID				= hton64(pRequest->LocalCaGUID);
	pRequest->Reserved2					=	0;
	pRequest->LocalQKey					= hton32(pRequest->LocalQKey);

	pRequest->u1.AsUint32				= hton32(pRequest->u1.AsUint32);
	pRequest->u2.AsUint32				= hton32(pRequest->u2.AsUint32);
	pRequest->u3.AsUint32				= hton32(pRequest->u3.AsUint32);
	pRequest->u4.AsUint32				= hton32(pRequest->u4.AsUint32);
	
	pRequest->PartitionKey				= hton16(pRequest->PartitionKey);

	pRequest->PrimaryLocalLID			= hton16(pRequest->PrimaryLocalLID);
	pRequest->PrimaryRemoteLID			= hton16(pRequest->PrimaryRemoteLID);

	BSWAP_IB_GID(&pRequest->PrimaryLocalGID);
	BSWAP_IB_GID(&pRequest->PrimaryRemoteGID);

	pRequest->u5.AsUint32				= hton32(pRequest->u5.AsUint32);

	pRequest->AlternateLocalLID			= hton16(pRequest->AlternateLocalLID);
	pRequest->AlternateRemoteLID		= hton16(pRequest->AlternateRemoteLID);

	BSWAP_IB_GID(&pRequest->AlternateLocalGID);
	BSWAP_IB_GID(&pRequest->AlternateRemoteGID);

	pRequest->u6.AsUint32				= hton32(pRequest->u6.AsUint32);

} // ConvertREQToNetworkOrder()

void
ConvertREPToNetworkOrder(CMM_REP* pReply)
{
	pReply->LocalCommID				= hton32(pReply->LocalCommID);
	pReply->RemoteCommID			= hton32(pReply->RemoteCommID);
	pReply->LocalQKey				= hton32(pReply->LocalQKey);

	pReply->u1.AsUint32				= hton32(pReply->u1.AsUint32);
	pReply->u2.AsUint32				= hton32(pReply->u2.AsUint32);
	pReply->u3.AsUint32				= hton32(pReply->u3.AsUint32);


	pReply->LocalCaGUID				= hton64(pReply->LocalCaGUID);

} // ConvertREPToNetworkOrder()

void
ConvertLAPToNetworkOrder(CMM_LAP* pLap)
{
	pLap->LocalCommID				= hton32(pLap->LocalCommID);
	pLap->RemoteCommID				= hton32(pLap->RemoteCommID);

	pLap->u1.AsUint32				= hton32(pLap->u1.AsUint32);
	pLap->AlternateLocalLID			= hton16(pLap->AlternateLocalLID);
	pLap->AlternateRemoteLID		= hton16(pLap->AlternateRemoteLID);
	BSWAP_IB_GID(&pLap->AlternateLocalGID);
	BSWAP_IB_GID(&pLap->AlternateRemoteGID);
	pLap->u2.AsUint32				= hton32(pLap->u2.AsUint32);
} // ConvertLAPToNetworkOrder()

void
ConvertToNetworkOrder(CM_MAD*pMad)
{
	uint32	AttrID=0;

	MAD_GET_ATTRIB_ID(pMad, &AttrID);

	switch (AttrID)
	{
	case MCLASS_ATTRIB_ID_CLASS_PORT_INFO:

		BSWAP_IB_CLASS_PORT_INFO((IB_CLASS_PORT_INFO*)&pMad->payload.ClassPortInfo);
		break;

	case MCLASS_ATTRIB_ID_REQ:

		ConvertREQToNetworkOrder(&pMad->payload.REQ);
		break;

	case MCLASS_ATTRIB_ID_REP:

		ConvertREPToNetworkOrder(&pMad->payload.REP);
		break;

	case MCLASS_ATTRIB_ID_RTU:

		pMad->payload.RTU.LocalCommID	= hton32(pMad->payload.RTU.LocalCommID);
		pMad->payload.RTU.RemoteCommID	= hton32(pMad->payload.RTU.RemoteCommID);

		break;

	case MCLASS_ATTRIB_ID_REJ:
			
		pMad->payload.REJ.LocalCommID	= hton32(pMad->payload.REJ.LocalCommID);
		pMad->payload.REJ.RemoteCommID	= hton32(pMad->payload.REJ.RemoteCommID);
	
		pMad->payload.REJ.Reason		= hton16(pMad->payload.REJ.Reason);

		break;

	case MCLASS_ATTRIB_ID_DREQ:
			
		pMad->payload.DREQ.LocalCommID	= hton32(pMad->payload.DREQ.LocalCommID);
		pMad->payload.DREQ.RemoteCommID	= hton32(pMad->payload.DREQ.RemoteCommID);
		pMad->payload.DREQ.u.AsUint32 = hton32(pMad->payload.DREQ.u.AsUint32);

		break;

	case MCLASS_ATTRIB_ID_DREP:
			
		pMad->payload.DREP.LocalCommID	= hton32(pMad->payload.DREP.LocalCommID);
		pMad->payload.DREP.RemoteCommID	= hton32(pMad->payload.DREP.RemoteCommID);

		break;

	case MCLASS_ATTRIB_ID_SIDR_REQ:

		pMad->payload.SIDR_REQ.RequestID	= hton32(pMad->payload.SIDR_REQ.RequestID);
		pMad->payload.SIDR_REQ.PartitionKey	= hton16(pMad->payload.SIDR_REQ.PartitionKey);
		pMad->payload.SIDR_REQ.ServiceID	= hton64(pMad->payload.SIDR_REQ.ServiceID);

		break;

	case MCLASS_ATTRIB_ID_SIDR_RESP:
			
		pMad->payload.SIDR_RESP.RequestID	= hton32(pMad->payload.SIDR_RESP.RequestID);

		pMad->payload.SIDR_RESP.u.AsUint32	= hton32(pMad->payload.SIDR_RESP.u.AsUint32);
		pMad->payload.SIDR_RESP.ServiceID	= hton64(pMad->payload.SIDR_RESP.ServiceID);

		pMad->payload.SIDR_RESP.QKey		= hton32(pMad->payload.SIDR_RESP.QKey);

		break;

	case MCLASS_ATTRIB_ID_MRA:

		pMad->payload.MRA.LocalCommID	= hton32(pMad->payload.MRA.LocalCommID);
		pMad->payload.MRA.RemoteCommID	= hton32(pMad->payload.MRA.RemoteCommID);

		break;

	case MCLASS_ATTRIB_ID_LAP:

		ConvertLAPToNetworkOrder(&pMad->payload.LAP);
		break;

	case MCLASS_ATTRIB_ID_APR:

		pMad->payload.APR.LocalCommID	= hton32(pMad->payload.APR.LocalCommID);
		pMad->payload.APR.RemoteCommID	= hton32(pMad->payload.APR.RemoteCommID);
		break;

	default:
		break;


	} // switch()

	return;

} // ConvertToNetworkOrder()

void
ConvertToHostOrder(CM_MAD*	pMad)
{
	ConvertToNetworkOrder(pMad);
} // ConvertToHostOrder()

#if defined( DBG ) || defined( IB_DEBUG )

void
DumpClassPortInfo(const IB_CLASS_PORT_INFO* pClassPortInfo)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpClassPortInfo);

	_DBG_INFO(("\t\tClassPortInfo.BaseVersion = 0x%x\n", pClassPortInfo->BaseVersion));
	_DBG_INFO(("\t\tClassPortInfo.ClassVersion = 0x%x\n", pClassPortInfo->ClassVersion));
	_DBG_INFO(("\t\tClassPortInfo.CapMask = 0x%x\n", pClassPortInfo->CapMask));
	_DBG_INFO(("\t\tClassPortInfo.RespTimeValue = 0x%x\n", pClassPortInfo->u1.s.RespTimeValue));
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} // DumpClassPortInfo()
	
void
DumpREQ(const CMM_REQ* pREQ)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpReq);

	_DBG_INFO(("\t\tREQ.LocalCommID = 0x%x\n", pREQ->LocalCommID));
	_DBG_INFO(("\t\tREQ.ServiceID = 0x%"PRIx64"\n", pREQ->ServiceID));
	_DBG_INFO(("\t\tREQ.LocalCaGUID = 0x%"PRIx64"\n", pREQ->LocalCaGUID));
	_DBG_INFO(("\t\tREQ.LocalQKey = 0x%x\n", pREQ->LocalQKey));
	
	_DBG_INFO(("\t\tREQ.LocalQPN = 0x%x\n", pREQ->u1.s.LocalQPN));
	_DBG_INFO(("\t\tREQ.OfferedResponderResources = 0x%x\n", pREQ->u1.s.OfferedResponderResources));
	_DBG_INFO(("\t\tREQ.LocalEECN = 0x%x\n", pREQ->u2.s.LocalEECN));
	_DBG_INFO(("\t\tREQ.OfferedInitiatorDepth = 0x%x\n", pREQ->u2.s.OfferedInitiatorDepth));
	_DBG_INFO(("\t\tREQ.RemoteEECN = 0x%x\n", pREQ->u3.s.RemoteEECN));
	_DBG_INFO(("\t\tREQ.RemoteCMTimeout = 0x%x\n", pREQ->u3.s.RemoteCMTimeout));
	_DBG_INFO(("\t\tREQ.TransportServiceType = 0x%x\n", pREQ->u3.s.TransportServiceType));
	_DBG_INFO(("\t\tREQ.EndToEndFlowControl = 0x%x\n", pREQ->u3.s.EndToEndFlowControl));

	_DBG_INFO(("\t\tREQ.StartingPSN = 0x%x\n", pREQ->u4.s.StartingPSN));
	_DBG_INFO(("\t\tREQ.LocalCMTimeout = 0x%x\n", pREQ->u4.s.LocalCMTimeout));
	_DBG_INFO(("\t\tREQ.RetryCount = 0x%x\n", pREQ->u4.s.RetryCount));
	_DBG_INFO(("\t\tREQ.PartitionKey = 0x%x\n", pREQ->PartitionKey));
	_DBG_INFO(("\t\tREQ.PathMTU = 0x%x\n", pREQ->PathMTU));
	_DBG_INFO(("\t\tREQ.RdcExists = 0x%x\n", pREQ->RdcExists));
	_DBG_INFO(("\t\tREQ.RnRRetryCount = 0x%x\n", pREQ->RnRRetryCount));
	_DBG_INFO(("\t\tREQ.MaxCMRetries = 0x%x\n", pREQ->MaxCMRetries));

	_DBG_INFO(("\t\tREQ.PrimaryLocalLID = 0x%x\n", pREQ->PrimaryLocalLID));
	_DBG_INFO(("\t\tREQ.PrimaryLocalGID = Subnet Prefix (0x%"PRIx64")  Interface ID(0x%"PRIx64")\n",
				pREQ->PrimaryLocalGID.Type.Global.SubnetPrefix, pREQ->PrimaryLocalGID.Type.Global.InterfaceID));

	_DBG_INFO(("\t\tREQ.PrimaryRemoteLID = 0x%x\n", pREQ->PrimaryRemoteLID));
	_DBG_INFO(("\t\tREQ.PrimaryRemoteGID = Subnet Prefix (0x%"PRIx64")  Interface ID(0x%"PRIx64")\n",
				pREQ->PrimaryRemoteGID.Type.Global.SubnetPrefix, pREQ->PrimaryRemoteGID.Type.Global.InterfaceID));
	_DBG_INFO(("\t\tREQ.PrimaryFlowLabel = 0x%x\n", pREQ->u5.s.PrimaryFlowLabel));
	_DBG_INFO(("\t\tREQ.PrimaryPacketRate = 0x%x\n", pREQ->u5.s.PrimaryPacketRate));
	_DBG_INFO(("\t\tREQ.PrimaryTrafficClass = 0x%x\n", pREQ->PrimaryTrafficClass));
	_DBG_INFO(("\t\tREQ.PrimaryHopLimit = 0x%x\n", pREQ->PrimaryHopLimit));
	_DBG_INFO(("\t\tREQ.PrimarySL = 0x%x\n", pREQ->PrimarySL));
	_DBG_INFO(("\t\tREQ.PrimarySubnetLocal = 0x%x\n", pREQ->PrimarySubnetLocal));
	_DBG_INFO(("\t\tREQ.PrimaryLocalAckTimeout = 0x%x\n", pREQ->PrimaryLocalAckTimeout));

	if (pREQ->AlternateLocalLID)
	{
		_DBG_INFO(("\t\tREQ.AlternateLocalLID = 0x%x\n", pREQ->AlternateLocalLID));
		_DBG_INFO(("\t\tREQ.AlternateLocalGID = Subnet Prefix (0x%"PRIx64")  Interface ID(0x%"PRIx64")\n",
				pREQ->AlternateLocalGID.Type.Global.SubnetPrefix, pREQ->AlternateLocalGID.Type.Global.InterfaceID));

		_DBG_INFO(("\t\tREQ.AlternateRemoteLID = 0x%x\n", pREQ->AlternateRemoteLID));
		_DBG_INFO(("\t\tREQ.AlternateRemoteGID = Subnet Prefix (0x%"PRIx64")  Interface ID(0x%"PRIx64")\n",
				pREQ->AlternateRemoteGID.Type.Global.SubnetPrefix, pREQ->AlternateRemoteGID.Type.Global.InterfaceID));
		_DBG_INFO(("\t\tREQ.AlternateFlowLabel = 0x%x\n", pREQ->u6.s.AlternateFlowLabel));
		_DBG_INFO(("\t\tREQ.AlternatePacketRate = 0x%x\n", pREQ->u6.s.AlternatePacketRate));
		_DBG_INFO(("\t\tREQ.AlternateTrafficClass = 0x%x\n", pREQ->AlternateTrafficClass));
		_DBG_INFO(("\t\tREQ.AlternateHopLimit = 0x%x\n", pREQ->AlternateHopLimit));
		_DBG_INFO(("\t\tREQ.AlternateSL = 0x%x\n", pREQ->AlternateSL));
		_DBG_INFO(("\t\tREQ.AlternateSubnetLocal = 0x%x\n", pREQ->AlternateSubnetLocal));
		_DBG_INFO(("\t\tREQ.AlternateLocalAckTimeout = 0x%x\n", pREQ->AlternateLocalAckTimeout));
	}
         _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} // DumpREQ()

void 
DumpReqInfo(const CM_REQUEST_INFO* pReqInfo)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpReqInfo);
	_DBG_INFO(("\t\tRequestInfo.PathInfo.bSubnetLocal = 0x%x\n", pReqInfo->PathInfo.bSubnetLocal));
	_DBG_INFO(("\t\tRequestInfo.PathInfo.Path.SLID = 0x%x\n", pReqInfo->PathInfo.Path.SLID));
	_DBG_INFO(("\t\tRequestInfo.PathInfo.Path.DLID = 0x%x\n", pReqInfo->PathInfo.Path.DLID));

	_DBG_INFO(("\t\tRequestInfo.PathInfo.Path.u1.s.FlowLabel = 0x%x\n", pReqInfo->PathInfo.Path.u1.s.FlowLabel));
	_DBG_INFO(("\t\tRequestInfo.PathInfo.Path.u1.s.HopLimit = 0x%x\n", pReqInfo->PathInfo.Path.u1.s.HopLimit));
	_DBG_INFO(("\t\tRequestInfo.PathInfo.Path.TClass = 0x%x\n", pReqInfo->PathInfo.Path.TClass));
	_DBG_INFO(("\t\tRequestInfo.PathInfo.Path.u2.s.SL = 0x%x\n", pReqInfo->PathInfo.Path.u2.s.SL));
	_DBG_INFO(("\t\tRequestInfo.PathInfo.Path.Rate = 0x%x\n", pReqInfo->PathInfo.Path.Rate));
	_DBG_INFO(("\t\tRequestInfo.PathInfo.Path.P_Key = 0x%x\n", pReqInfo->PathInfo.Path.P_Key));
	_DBG_INFO(("\t\tRequestInfo.PathInfo.Path.Mtu = 0x%x\n", pReqInfo->PathInfo.Path.Mtu));
	_DBG_INFO(("\t\tRequestInfo.PathInfo.Path.PktLifeTime = 0x%x\n", pReqInfo->PathInfo.Path.PktLifeTime));
	
	_DBG_INFO(("\t\tRequestInfo.CEPInfo.CaGUID = 0x%"PRIx64"\n", pReqInfo->CEPInfo.CaGUID));
	_DBG_INFO(("\t\tRequestInfo.CEPInfo.EndToEndFlowControl = 0x%x\n", pReqInfo->CEPInfo.EndToEndFlowControl));
	//_DBG_INFO(("\t\tRequestInfo.CEPInfo.PortGUID = 0x%"PRIx64"\n", pReqInfo->CEPInfo.PortGUID));
	_DBG_INFO(("\t\tRequestInfo.CEPInfo.StartingPSN = 0x%x\n", pReqInfo->CEPInfo.StartingPSN));
	_DBG_INFO(("\t\tRequestInfo.CEPInfo.QPN = 0x%x\n", pReqInfo->CEPInfo.QPN));
	_DBG_INFO(("\t\tRequestInfo.CEPInfo.QKey = 0x%x\n", pReqInfo->CEPInfo.QKey));
	_DBG_INFO(("\t\tRequestInfo.CEPInfo.RetryCount = 0x%x\n", pReqInfo->CEPInfo.RetryCount));
	_DBG_INFO(("\t\tRequestInfo.CEPInfo.RnrRetryCount = 0x%x\n", pReqInfo->CEPInfo.RnrRetryCount));
	_DBG_INFO(("\t\tRequestInfo.CEPInfo.AckTimeout = 0x%x\n", pReqInfo->CEPInfo.AckTimeout));
	_DBG_INFO(("\t\tRequestInfo.CEPInfo.AlternateAckTimeout = 0x%x\n", pReqInfo->CEPInfo.AlternateAckTimeout));
	_DBG_INFO(("\t\tRequestInfo.CEPInfo.OfferedResponderResources = 0x%x\n", pReqInfo->CEPInfo.OfferedResponderResources));
	_DBG_INFO(("\t\tRequestInfo.CEPInfo.OfferedInitiatorDepth = 0x%x\n", pReqInfo->CEPInfo.OfferedInitiatorDepth));
	_DBG_INFO(("\t\tRequestInfo.CEPInfo.LocalEECN = 0x%x\n", pReqInfo->CEPInfo.LocalEECN));
	_DBG_INFO(("\t\tRequestInfo.CEPInfo.RemoteEECN = 0x%x\n", pReqInfo->CEPInfo.RemoteEECN));

         _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} // DumpReqInfo()


void
DumpREP(const CMM_REP* pREP)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpRep);
	_DBG_INFO(("\t\tREP.LocalCommID = 0x%x\n", pREP->LocalCommID));
	_DBG_INFO(("\t\tREP.RemoteCommID = 0x%x\n", pREP->RemoteCommID));
	_DBG_INFO(("\t\tREP.LocalQKey = 0x%x\n", pREP->LocalQKey));
	_DBG_INFO(("\t\tREP.LocalQPN = 0x%x\n", pREP->u1.s.LocalQPN));
	_DBG_INFO(("\t\tREP.LocalEECN = 0x%x\n", pREP->u2.s.LocalEECN));
	_DBG_INFO(("\t\tREP.StartingPSN = 0x%x\n", pREP->u3.s.StartingPSN));
	_DBG_INFO(("\t\tREP.ArbResponderResources = 0x%x\n", pREP->ArbResponderResources));
	_DBG_INFO(("\t\tREP.ArbInitiatorDepth = 0x%x\n", pREP->ArbInitiatorDepth));
	_DBG_INFO(("\t\tREP.TargetAckDelay = 0x%x\n", pREP->TargetAckDelay));
	_DBG_INFO(("\t\tREP.EndToEndFlowControl = 0x%x\n", pREP->EndToEndFlowControl));
	_DBG_INFO(("\t\tREP.RnRRetryCount = 0x%x\n", pREP->RnRRetryCount));
	_DBG_INFO(("\t\tREP.LocalCaGUID = 0x%"PRIx64"\n", pREP->LocalCaGUID));

         _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} //DumpREP()

void 
DumpRepInfo(const CM_REPLY_INFO* pRepInfo)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpRepInfo);
	_DBG_INFO(("\t\tReplyInfo.QKey = 0x%x\n", pRepInfo->QKey));
	_DBG_INFO(("\t\tReplyInfo.QPN = %d\n", pRepInfo->QPN));
	_DBG_INFO(("\t\tReplyInfo.EECN = %d\n", pRepInfo->EECN));
	_DBG_INFO(("\t\tReplyInfo.StartingPSN = %d\n", pRepInfo->StartingPSN));
	_DBG_INFO(("\t\tReplyInfo.ArbResponderResources = %d\n", pRepInfo->ArbResponderResources));
	_DBG_INFO(("\t\tReplyInfo.ArbInitiatorDepth = %d\n", pRepInfo->ArbInitiatorDepth));
	_DBG_INFO(("\t\tReplyInfo.TargetAckDelay = %d\n", pRepInfo->TargetAckDelay));
	_DBG_INFO(("\t\tReplyInfo.FailoverAccepted = %d\n", pRepInfo->FailoverAccepted));
	_DBG_INFO(("\t\tReplyInfo.EndToEndFlowControl = %d\n", pRepInfo->EndToEndFlowControl));
	_DBG_INFO(("\t\tReplyInfo.RnRRetryCount = %d\n", pRepInfo->RnRRetryCount));
	_DBG_INFO(("\t\tReplyInfo.CaGUID = 0x%"PRIx64"\n", pRepInfo->CaGUID));
        _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} // DumpRepInfo()

void DumpREJ(const CMM_REJ* pREJ)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpRej);
	_DBG_INFO(("\t\tREJ.LocalCommID = 0x%x\n", pREJ->LocalCommID));
	_DBG_INFO(("\t\tREJ.RemoteCommID = 0x%x\n", pREJ->RemoteCommID));
	_DBG_INFO(("\t\tREJ.MsgRejected = 0x%x\n", pREJ->MsgRejected));
	_DBG_INFO(("\t\tREJ.RejectInfoLen = 0x%x\n", pREJ->RejectInfoLen));
	_DBG_INFO(("\t\tREJ.Reason = 0x%x\n", pREJ->Reason));

        _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} // DumpREJ()

void 
DumpRejInfo(const CM_REJECT_INFO* pRejectInfo)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpRejInfo);
	_DBG_INFO(("\t\tRejectInfo.RejectInfoLen = %d\n", pRejectInfo->RejectInfoLen));
	_DBG_INFO(("\t\tRejectInfo.Reason = %d\n", pRejectInfo->Reason));
        _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} // DumpRejInfo()

void
DumpLAP(const CMM_LAP* pLAP)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpLAP);
	_DBG_INFO(("\t\tLAP.LocalCommID = 0x%x\n", pLAP->LocalCommID));
	_DBG_INFO(("\t\tLAP.RemoteCommID = 0x%x\n", pLAP->RemoteCommID));
    _DBG_INFO(("\t\tLAP.RemoteQPNorEECN = 0x%x\n", pLAP->u1.s.RemoteQPNorEECN));
	_DBG_INFO(("\t\tLAP.RemoteCMTimeout = 0x%x\n", pLAP->u1.s.RemoteCMTimeout));
	_DBG_INFO(("\t\tLAP.AlternateLocalLID = 0x%x\n", pLAP->AlternateLocalLID));
	_DBG_INFO(("\t\tLAP.AlternateLocalGID = Subnet Prefix (0x%"PRIx64")  Interface ID(0x%"PRIx64")\n",
			pLAP->AlternateLocalGID.Type.Global.SubnetPrefix, pLAP->AlternateLocalGID.Type.Global.InterfaceID));

	_DBG_INFO(("\t\tLAP.AlternateRemoteLID = 0x%x\n", pLAP->AlternateRemoteLID));
	_DBG_INFO(("\t\tLAP.AlternateRemoteGID = Subnet Prefix (0x%"PRIx64")  Interface ID(0x%"PRIx64")\n",
			pLAP->AlternateRemoteGID.Type.Global.SubnetPrefix, pLAP->AlternateRemoteGID.Type.Global.InterfaceID));
	_DBG_INFO(("\t\tLAP.AlternateFlowLabel = 0x%x\n", pLAP->u2.s.AlternateFlowLabel));
	_DBG_INFO(("\t\tLAP.AlternateTrafficClass = 0x%x\n", pLAP->u2.s.AlternateTrafficClass));
	_DBG_INFO(("\t\tLAP.AlternateHopLimit = 0x%x\n", pLAP->AlternateHopLimit));
	_DBG_INFO(("\t\tLAP.AlternatePacketRate = 0x%x\n", pLAP->AlternatePacketRate));
	_DBG_INFO(("\t\tLAP.AlternateSL = 0x%x\n", pLAP->AlternateSL));
	_DBG_INFO(("\t\tLAP.AlternateSubnetLocal = 0x%x\n", pLAP->AlternateSubnetLocal));
	_DBG_INFO(("\t\tLAP.AlternateLocalAckTimeout = 0x%x\n", pLAP->AlternateLocalAckTimeout));
        _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}

void 
DumpLapInfo(const CM_ALTPATH_INFO* pLapInfo)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpLapInfo);
	_DBG_INFO(("\t\tLapInfo.AlternateAckTimeout = 0x%x\n", pLapInfo->AlternateAckTimeout));
	_DBG_INFO(("\t\tLapInfo.AlternatePathInfo.bSubnetLocal = 0x%x\n", pLapInfo->AlternatePathInfo.bSubnetLocal));
	_DBG_INFO(("\t\tLapInfo.AlternatePathInfo.Path.SLID = 0x%x\n", pLapInfo->AlternatePathInfo.Path.SLID));
	_DBG_INFO(("\t\tLapInfo.AlternatePathInfo.Path.DLID = 0x%x\n", pLapInfo->AlternatePathInfo.Path.DLID));

	_DBG_INFO(("\t\tLapInfo.AlternatePathInfo.Path.u1.s.FlowLabel = 0x%x\n", pLapInfo->AlternatePathInfo.Path.u1.s.FlowLabel));
	_DBG_INFO(("\t\tLapInfo.AlternatePathInfo.Path.u1.s.HopLimit = 0x%x\n", pLapInfo->AlternatePathInfo.Path.u1.s.HopLimit));
	_DBG_INFO(("\t\tLapInfo.AlternatePathInfo.Path.TClass = 0x%x\n", pLapInfo->AlternatePathInfo.Path.TClass));
	_DBG_INFO(("\t\tLapInfo.AlternatePathInfo.Path.u2.s.SL = 0x%x\n", pLapInfo->AlternatePathInfo.Path.u2.s.SL));
	_DBG_INFO(("\t\tLapInfo.AlternatePathInfo.Path.Rate = 0x%x\n", pLapInfo->AlternatePathInfo.Path.Rate));
	_DBG_INFO(("\t\tLapInfo.AlternatePathInfo.Path.P_Key = 0x%x\n", pLapInfo->AlternatePathInfo.Path.P_Key));
	_DBG_INFO(("\t\tLapInfo.AlternatePathInfo.Path.Mtu = 0x%x\n", pLapInfo->AlternatePathInfo.Path.Mtu));
	_DBG_INFO(("\t\tLapInfo.AlternatePathInfo.Path.PktLifeTime = 0x%x\n", pLapInfo->AlternatePathInfo.Path.PktLifeTime));
        _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}

void DumpAPR(const CMM_APR* pAPR)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpAPR);
	_DBG_INFO(("\t\tAPR.LocalCommID = 0x%x\n", pAPR->LocalCommID));
	_DBG_INFO(("\t\tAPR.RemoteCommID = 0x%x\n", pAPR->RemoteCommID));
	_DBG_INFO(("\t\tAPR.APStatus = 0x%x\n", pAPR->APStatus));
	_DBG_INFO(("\t\tAPR.AddInfoLen = 0x%x\n", pAPR->AddInfoLen));
        _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} // DumpAPR()

void 
DumpAprInfo(const CM_ALTPATH_REPLY_INFO* pAprInfo)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpAprInfo);
	_DBG_INFO(("\t\tAprInfo.u.AddInfo.Len = %d\n", pAprInfo->u.AddInfo.Len));
	_DBG_INFO(("\t\tAprInfo.APStatus = %d\n", pAprInfo->APStatus));
        _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} // DumpAprInfo()

void DumpDREQ(const CMM_DREQ* pDREQ)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpDREQ);
	_DBG_INFO(("\t\tDREQ.LocalCommID = 0x%x\n", pDREQ->LocalCommID));
	_DBG_INFO(("\t\tDREQ.RemoteCommID = 0x%x\n", pDREQ->RemoteCommID));
	_DBG_INFO(("\t\tDREQ.RemoteQPNorEECN = 0x%x\n", pDREQ->u.s.RemoteQPNorEECN));

        _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} // DumpDREQ()

void 
DumpDReqInfo(const CM_DREQUEST_INFO* pDReqInfo)
{
} // DumpDReqInfo()

void DumpSIDR_REQ(const CMM_SIDR_REQ* pSIDR_REQ)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpSIDR_REQ);
	_DBG_INFO(("\t\tSIDR_REQ.RequestID = 0x%x\n", pSIDR_REQ->RequestID));
	_DBG_INFO(("\t\tSIDR_REQ.PartitionKey = 0x%04x\n", pSIDR_REQ->PartitionKey));
	_DBG_INFO(("\t\tSIDR_REQ.ServiceID = 0x%"PRIx64"\n", pSIDR_REQ->ServiceID));

        _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} // DumpSIDR_REQ()

void DumpSidrReqInfo(const SIDR_REQ_INFO* pSidrReqInfo)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpSidrReqInfo);
	_DBG_INFO(("\t\tSidrReqInfo.SID = 0x%"PRIx64"\n", pSidrReqInfo->SID));
	_DBG_INFO(("\t\tSidrReqInfo.PartitionKey = 0x%04x\n", pSidrReqInfo->PartitionKey));
	_DBG_INFO(("\t\tSidrReqInfo.PortGuid = 0x%"PRIx64"\n", pSidrReqInfo->PortGuid));
	_DBG_INFO(("\t\tSidrReqInfo.PathInfo.Path.DLID = 0x%x\n", pSidrReqInfo->PathInfo.Path.DLID));
	_DBG_INFO(("\t\tSidrReqInfo.PathInfo.Path.u2.s.SL = 0x%x\n", pSidrReqInfo->PathInfo.Path.u2.s.SL));
	_DBG_INFO(("\t\tSidrReqInfo.PathInfo.Path.Rate = 0x%x\n", pSidrReqInfo->PathInfo.Path.Rate));

        _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} // DumpSidrReqInfo()

void DumpSIDR_RESP(const CMM_SIDR_RESP* pSIDR_RESP)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpSIDR_RESP);
	_DBG_INFO(("\t\tSIDR_RESP.RequestID = 0x%x\n", pSIDR_RESP->RequestID));
	_DBG_INFO(("\t\tSIDR_RESP.Status = 0x%x\n", pSIDR_RESP->Status));
	_DBG_INFO(("\t\tSIDR_RESP.AddInfoLen = 0x%x\n", pSIDR_RESP->AddInfoLen));
	_DBG_INFO(("\t\tSIDR_RESP.QPN = 0x%x\n", pSIDR_RESP->u.s.QPN));
	_DBG_INFO(("\t\tSIDR_RESP.ServiceID = 0x%"PRIx64"\n", pSIDR_RESP->ServiceID));
	_DBG_INFO(("\t\tSIDR_RESP.QKey = 0x%x\n", pSIDR_RESP->QKey));

        _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} // DumpSIDR_RESP()

void DumpSidrRespInfo(const SIDR_RESP_INFO* pSidrRespInfo)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DumpSidrRespInfo);
	_DBG_INFO(("\t\tSidrRespInfo.QPN = 0x%x\n", pSidrRespInfo->QPN));
	_DBG_INFO(("\t\tSidrRespInfo.Status = 0x%x\n", pSidrRespInfo->Status));
	_DBG_INFO(("\t\tSidrRespInfo.QKey = 0x%x\n", pSidrRespInfo->QKey));

        _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} // DumpSidrRespInfo()

#endif // #if defined( DBG ) || defined( IB_DEBUG )
