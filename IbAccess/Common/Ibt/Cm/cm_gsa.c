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

static FSTATUS
ProcessRecvCmMad(
	IN CM_MAD*	pMad,
	IN IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessClassPortInfo(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessReq(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessReqPassive(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_MAD*					pMad,
	IN IBT_DGRM_ELEMENT*		pRemoteDgrm
	);

static FSTATUS
ProcessReqPeer(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_MAD*					pMad,
	IN IBT_DGRM_ELEMENT*		pRemoteDgrm,
	IN CM_CEP_OBJECT*			pCEPReq
	);

static FSTATUS
SaveReqToPassiveCEP(
	IN CM_CEP_OBJECT*			pCEP,
	IN CMM_REQ*					pREQ,
	IN EUI64					CaGUID,
	IN EUI64					PortGuid,	// port which received REQ
	OUT CM_REJECTION_CODE		*RejReason	// only returned on failure
	);

static FSTATUS
ProcessReqDup(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_MAD*					pMad,
	IN IBT_DGRM_ELEMENT*		pRemoteDgrm
	);

static FSTATUS
ProcessReqStale(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_MAD*					pMad,
	IN IBT_DGRM_ELEMENT*		pRemoteDgrm
	);

static FSTATUS
ProcessRep(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessRtu(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessRtuPassive(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_MAD*					pMad,
	IN IBT_DGRM_ELEMENT*		pRemoteDgrm
	);

static FSTATUS
ProcessRejReq(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessRejRep(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessMraReq(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessMraRep(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessMraLap(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessLap(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessApr(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessDreq(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessDreqCep(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_MAD*					pMad,
	IN IBT_DGRM_ELEMENT*		pRemoteDgrm
	);

static FSTATUS
ProcessDrep(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessSidrReq(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ProcessSidrResp(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	);

static FSTATUS
ResendREQ(
	IN CM_CEP_OBJECT*	pCEP
	);

static FSTATUS
ResendREP(
	IN CM_CEP_OBJECT*	pCEP
	);

//
// DgrmDump
//
// Show the message type Sent/Received
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
//	None
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
void
DgrmDump(IBT_DGRM_ELEMENT* pDgrm, char* pStr, boolean bRecv)
{
	CM_MAD* pMad=NULL;

	if (bRecv)
		pMad = (CM_MAD*)GsiDgrmGetRecvMad(pDgrm);
	else
		pMad = (CM_MAD*)GsiDgrmGetSendMad(pDgrm);

	switch (pMad->common.AttributeID)
	{
	case MCLASS_ATTRIB_ID_CLASS_PORT_INFO:
		_DBG_INFO(("<dgrm 0x%p> *** ClassPortInfo %s ***\n",
				_DBG_PTR(pDgrm), _DBG_PTR(pStr)));
		break;

	case MCLASS_ATTRIB_ID_REQ:
		_DBG_INFO(("<dgrm 0x%p> *** REQ %s ***\n",
				_DBG_PTR(pDgrm), _DBG_PTR(pStr)));
		break;

	case MCLASS_ATTRIB_ID_MRA:
		_DBG_INFO(("<dgrm 0x%p> *** MRA %s ***\n",
				_DBG_PTR(pDgrm), _DBG_PTR(pStr)));
		break;

	case MCLASS_ATTRIB_ID_REJ:
		_DBG_INFO(("<dgrm 0x%p> *** REJ %s ***\n",
				_DBG_PTR(pDgrm), _DBG_PTR(pStr)));
		break;

	case MCLASS_ATTRIB_ID_REP:
		_DBG_INFO(("<dgrm 0x%p> *** REP %s ***\n",
				_DBG_PTR(pDgrm), _DBG_PTR(pStr)));
		break;

	case MCLASS_ATTRIB_ID_RTU:
		_DBG_INFO(("<dgrm 0x%p> *** RTU %s ***\n",
				_DBG_PTR(pDgrm), _DBG_PTR(pStr)));
		break;

	case MCLASS_ATTRIB_ID_DREQ:
		_DBG_INFO(("<dgrm 0x%p> *** DREQ %s ***\n",
				_DBG_PTR(pDgrm), _DBG_PTR(pStr)));
		break;

	case MCLASS_ATTRIB_ID_DREP:
		_DBG_INFO(("<dgrm 0x%p> *** DREP %s ***\n",
				_DBG_PTR(pDgrm), _DBG_PTR(pStr)));
		break;

	case MCLASS_ATTRIB_ID_SIDR_REQ:
		_DBG_INFO(("<dgrm 0x%p> *** SIDR REQ %s ***\n",
				_DBG_PTR(pDgrm), _DBG_PTR(pStr)));
		//MsgOut("*** SIDR REQ %s Compl ***, Remote Lid=0x%x\n", _DBG_PTR(pStr), pDgrm->RemoteLID);
		break;

	case MCLASS_ATTRIB_ID_SIDR_RESP:
		_DBG_INFO(("<dgrm 0x%p> *** SIDR RESP %s ***\n",
				_DBG_PTR(pDgrm), _DBG_PTR(pStr)));
		//MsgOut("*** SIDR RESP %s Compl ***, Remote Lid=0x%x\n", _DBG_PTR(pStr), pDgrm->RemoteLID);
		break;

	case MCLASS_ATTRIB_ID_LAP:
		_DBG_INFO(("<dgrm 0x%p> *** LAP %s ***\n",
				_DBG_PTR(pDgrm), _DBG_PTR(pStr)));
		break;

	case MCLASS_ATTRIB_ID_APR:
		_DBG_INFO(("<dgrm 0x%p> *** APR %s ***\n",
				_DBG_PTR(pDgrm), _DBG_PTR(pStr)));
		break;

	default:
		_DBG_INFO(("<dgrm 0x%p> *** UNKNOWN %s ***\n",
				_DBG_PTR(pDgrm), _DBG_PTR(pStr)));
		break;
	}

} // DgrmDump()

//
// VpdAsyncEventCallback
//
// VPD/VCA calls this routine when a RTU event is received on the receive QP
// or a Path Migrated event occurs
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
//	FSUCCESS - Indicates a successful load.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
#ifdef _CM_EVENT_ON_QP_

void 
VpdCompletionCallback(
	IN	void *CaContext, 
	IN	void *CqContext
	)
{

}

void
VpdAsyncEventCallback(
	IN	void				*CaContext, 
	IN IB_EVENT_RECORD		*EventRecord
	)
{
	CM_DEVICE_OBJECT*		pDevice=(CM_DEVICE_OBJECT*)CaContext;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, VpdAsyncEventCallback);

	_DBG_INFO(("<dev 0x%p> Received Async Event <context 0x%p, type 0x%x, code 0x%x\n", 
				_DBG_PTR(pDevice), _DBG_PTR(EventRecord->Context),
				EventRecord->EventType, EventRecord->EventCode));

	//
	// If we received a RTU event on the receive queue, we locate the appropriate
	// CEP object using the QPN and transition the CEP state to established state
	//
	switch ( EventRecord->EventType )
	{
	case AsyncEventRecvQ:
		if (EventRecord->EventCode == IB_AE_RQ_COMM_ESTABLISHED)
		{
			CM_END_POINT EndPoint;
			CM_MAP_ITEM* pMapEntry;

			EndPoint.CaGUID = pDevice->CaGUID;
			// special Async event for CM, QPN is in Context
			EndPoint.QPN = (uint32)(uintn)EventRecord->Context;
			EndPoint.EECN = 0;
			_DBG_INFO(("<dev 0x%p> Received RTU event on RQ <qpn %u, ca guid 0x%"PRIx64">\n",
							_DBG_PTR(pDevice), EndPoint.QPN, pDevice->CaGUID));
			SpinLockAcquire(&gCM->ListLock);
			pMapEntry = CM_MapGetCompare(&gCM->LocalEndPointMap, (uintn)&EndPoint,
							CepAddrLocalEndPointCompare);
			if (pMapEntry != CM_MapEnd(&gCM->LocalEndPointMap))
			{
				CM_CEP_OBJECT* pCEP;
				FSTATUS Status;

				pCEP = PARENT_STRUCT(pMapEntry, CM_CEP_OBJECT, LocalEndPointMapEntry);
				// Process as an RTU
				Status = ProcessRtuPassive(pCEP, NULL, NULL);
				if (Status == FSUCCESS)
				{
					_DBG_INFO(("<dev 0x%p> RTU event on RQ processed successfully <cep 0x%p qpn %u, ca guid 0x%"PRIx64">!\n",
						_DBG_PTR(pDevice), _DBG_PTR(pCEP),
						EndPoint.QPN, pDevice->CaGUID));
				} else if (Status != FINVALID_STATE) {
					_DBG_WARNING(("<dev 0x%p> RTU event on RQ dropped - unable to process it! <%s, qpn %u, ca guid 0x%"PRIx64" >\n",
						_DBG_PTR(pDevice), _DBG_PTR(FSTATUS_MSG(Status)),
						EndPoint.QPN, pDevice->CaGUID));
				}
			} else {
				_DBG_WARNING(("<dev 0x%p> RTU event on RQ dropped - no match on QPN and CA GUID <qpn %u, ca guid 0x%"PRIx64">!\n",
							_DBG_PTR(pDevice), EndPoint.QPN, pDevice->CaGUID));
			}

			SpinLockRelease(&gCM->ListLock);
		}
		break;
	// TBD - case AsyncEventEE:
#if 0
	// disabled, this could be helpful for Server side, where there is
	// no option.  However for Client side we want to allow client to
	// choose between iba_cm_migrated and iba_cm_migrated_reload
	case AsyncEventPathMigrated:
		{
			CM_END_POINT EndPoint;
			CM_MAP_ITEM* pMapEntry;

			EndPoint.CaGUID = pDevice->CaGUID;
			// special Async event for CM, QPN is in Context
			EndPoint.QPN = (uint32)(uintn)EventRecord->Context;
			EndPoint.EECN = 0;
			_DBG_INFO(("<dev 0x%p> Received Path Migrated event <qpn %u, ca guid 0x%"PRIx64">\n",
							_DBG_PTR(pDevice), EndPoint.QPN, pDevice->CaGUID));
			SpinLockAcquire(&gCM->ListLock);
			pMapEntry = CM_MapGetCompare(&gCM->LocalEndPointMap, (uintn)&EndPoint,
							CepAddrLocalEndPointCompare);
			if (pMapEntry != CM_MapEnd(&gCM->LocalEndPointMap))
			{
				CM_CEP_OBJECT* pCEP;
				FSTATUS Status;

				pCEP = PARENT_STRUCT(pMapEntry, CM_CEP_OBJECT, LocalEndPointMapEntry);
				//if (pCEP->Mode == CM_MODE_PASSIVE)
				//and adjust iba_cm_migrated to be a noop for CM_MODE_PASSIVE
				Status = MigratedA(pCEP);
				if (Status == FSUCCESS)
				{
					_DBG_INFO(("<dev 0x%p> Path Migrated event processed successfully <cep 0x%p qpn %u, ca guid 0x%"PRIx64">!\n",
						_DBG_PTR(pDevice), _DBG_PTR(pCEP),
						EndPoint.QPN, pDevice->CaGUID));
				} else if (Status != FINVALID_STATE) {
					_DBG_WARNING(("<dev 0x%p> Path Migrated event dropped - unable to process it! <%s, qpn %u, ca guid 0x%"PRIx64" >\n",
						_DBG_PTR(pDevice), _DBG_PTR(FSTATUS_MSG(Status)),
						EndPoint.QPN, pDevice->CaGUID));
				}
			} else {
				_DBG_WARNING(("<dev 0x%p> Path Migrated event dropped - no match on QPN and CA GUID <qpn %u, ca guid 0x%"PRIx64">!\n",
							_DBG_PTR(pDevice), EndPoint.QPN, pDevice->CaGUID));
			}

			SpinLockRelease(&gCM->ListLock);
		}
		break;
	// TBD - case AsyncEventEEPathMigrated:
#endif
	default:
		break;
	}

    _DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}
#endif


//
// GsaSendCompleteCallback
//
// GSA called this routine when a send datagram is completed.
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
//	FSUCCESS - Indicates a successful load.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//

void
GsaSendCompleteCallback(
	void* Context,
	IBT_DGRM_ELEMENT* pDgrmList
	)
{
	IBT_DGRM_ELEMENT* pDgrmElement=NULL;
	IBT_DGRM_ELEMENT* pNextDgrmElement=NULL;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, GsaSendCompleteCallback);

	// Return the dgrm to the pool if it is not owned
	SpinLockAcquire(&gCM->ListLock);

	pDgrmElement = pDgrmList;
	while (pDgrmElement)
	{
		pNextDgrmElement = (IBT_DGRM_ELEMENT*) pDgrmElement->Element.pNextElement;
		
		DgrmDump(pDgrmElement, "Sent", 0);

		CmDgrmClearInUse(pDgrmElement);
		pDgrmElement->Element.pNextElement = NULL;
		if (CmDgrmIsReleaseFlagSet(pDgrmElement))
		{
			_DBG_INFO(("<dgrm 0x%p> Datagram returned to pool <pool 0x%p>\n", 
						_DBG_PTR(pDgrmElement), _DBG_PTR(gCM->GSADgrmPoolHandle)));

			CmDgrmPut(pDgrmElement);
		} else {
			_DBG_INFO(("<dgrm 0x%p> Datagram marked send done\n",
					_DBG_PTR(pDgrmElement)));
		}
		pDgrmElement = pNextDgrmElement;
	}

	SpinLockRelease(&gCM->ListLock);
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
    return;
} // GsaSendCompleteCallback()


//////////////////////////////////////////////////////////////////////////
// GsaRecvCallback
//
// GSA called this routine when a datagram is received.
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
//	FSUCCESS - Indicates a successful load.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//

FSTATUS
GsaRecvCallback(
	void* Context,
	IBT_DGRM_ELEMENT* pDgrmList
	)
{
	IBT_DGRM_ELEMENT* pDgrmElement=NULL;
	IBT_DGRM_ELEMENT* pNextDgrmElement=NULL;

	CM_MAD* pMad=NULL;

	uint8 BaseVer=0;
	uint8 MgmtClass=0;
	uint8 ClassVer=0;

	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, GsaRecvCallback);
	
	ASSERT(pDgrmList);

	// Sanity checks
	if (Context != gCM)
	{
		_DBG_ERROR(("<cm 0x%p> Invalid CM Context passed in!!!\n", _DBG_PTR(Context)));
		_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,FCOMPLETED);
		return FCOMPLETED;
	}

	//
	// iterate thru each datagrams/mads in the list
	//
	for (pDgrmElement = pDgrmList;  pDgrmElement; pDgrmElement = pNextDgrmElement)
	{
		pNextDgrmElement = (IBT_DGRM_ELEMENT*) pDgrmElement->Element.pNextElement;

		// Basic checks
		/* MAD_BLOCK_SIZE(2048) + GRH (40) */
		if (pDgrmElement->Element.RecvByteCount > CM_DGRM_SIZE)
		{
			AtomicIncrementVoid(&gCM->Recv.Failed);
			_DBG_WARNING(("<dgrm 0x%p> Datagram passed more than %d bytes <len %d>\n",
							_DBG_PTR(pDgrmElement),
							MAD_BLOCK_SIZE,
							pDgrmElement->Element.RecvByteCount));
			continue;
		}

		pMad = (CM_MAD*)GsiDgrmGetRecvMad(pDgrmElement);
		if (!pMad)
		{
			AtomicIncrementVoid(&gCM->Recv.Failed);
			_DBG_WARNING(("<dgrm 0x%p> Empty MAD passed in!\n", _DBG_PTR(pDgrmElement)));
			continue;
		}

		MAD_GET_VERSION_INFO(pMad, &BaseVer, &MgmtClass, &ClassVer);


		if ( (BaseVer != IB_BASE_VERSION) || 
			 (MgmtClass != MCLASS_COMM_MGT) || 
			 (ClassVer != IB_COMM_MGT_CLASS_VERSION))
		{
			// BUGBUG IBTA 1.1 pg 623 12.11.2.1 says
			// if SIDR_REQ, we need to respond with
			// Sidr_Resp.Status = SRS_VERSION_NOT_SUPPORTED and put the highest
			// version we support in AddInfo.  Instead we discard here
			// ?? such as response optional??
			AtomicIncrementVoid(&gCM->Recv.Failed);
			_DBG_WARNING(("<mad 0x%p> Invalid base version, mgmt class or class version <bv %d mc %d cv %d lid 0x%x>\n", 
							_DBG_PTR(pMad), BaseVer, MgmtClass, ClassVer,
							pDgrmElement->RemoteLID));
			continue;
		}

		DgrmDump(pDgrmElement, "Received", 1);
		// Other mad verification should be done by GSA.
	
		ConvertMadToHostByteOrder(pMad);

		ProcessRecvCmMad(pMad, pDgrmElement);
	}

	// The pDgrmList would not be valid when this call returns

	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,FCOMPLETED);
	return FCOMPLETED;

} // GsaRecvCallback()


//////////////////////////////////////////////////////////////////////////
// ProcessRecvCmMad()
//
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
//	FSUCCESS - Indicates a successful load.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//

FSTATUS
ProcessRecvCmMad(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessRecvCmMad);

	SpinLockAcquire(&gCM->ListLock);

	switch (pMad->common.AttributeID)
	{
	case MCLASS_ATTRIB_ID_CLASS_PORT_INFO: // Class Port Info Received
		AtomicIncrementVoid(&gCM->Recv.ClassPortInfo);
		Status = ProcessClassPortInfo(pMad, pRemoteDgrm);
		break;

	case MCLASS_ATTRIB_ID_REQ: // REQ Received
		AtomicIncrementVoid(&gCM->Recv.Req);
		Status = ProcessReq(pMad, pRemoteDgrm);
		break;

	case MCLASS_ATTRIB_ID_REP: // REP Received
		AtomicIncrementVoid(&gCM->Recv.Rep);
		Status = ProcessRep(pMad, pRemoteDgrm);
		break;

	case MCLASS_ATTRIB_ID_RTU: // RTU received
		AtomicIncrementVoid(&gCM->Recv.Rtu);
		Status = ProcessRtu(pMad, pRemoteDgrm);
		break;

	case MCLASS_ATTRIB_ID_REJ: // REJ received
		if (pMad->payload.REJ.MsgRejected == CM_REJECT_REQUEST)
		{
			AtomicIncrementVoid(&gCM->Recv.RejReq);
			Status = ProcessRejReq(pMad, pRemoteDgrm);
		} else if (pMad->payload.REJ.MsgRejected == CM_REJECT_REPLY) {
			AtomicIncrementVoid(&gCM->Recv.RejRep);
			Status = ProcessRejRep(pMad, pRemoteDgrm);
		}
		// TBD MsgRejected can be unknown when remote side is rejecting due
		// to a timeout
		break;

	case MCLASS_ATTRIB_ID_MRA:
		if (pMad->payload.MRA.MsgMraed == CM_MRA_REQUEST)
		{
			AtomicIncrementVoid(&gCM->Recv.MraReq);
			Status = ProcessMraReq(pMad, pRemoteDgrm);
		} else if (pMad->payload.MRA.MsgMraed == CM_MRA_REPLY) {
			AtomicIncrementVoid(&gCM->Recv.MraRep);
			Status = ProcessMraRep(pMad, pRemoteDgrm);
		} else if (pMad->payload.MRA.MsgMraed == CM_MRA_LAP) {
			AtomicIncrementVoid(&gCM->Recv.MraLap);
			Status = ProcessMraLap(pMad, pRemoteDgrm);
		}
		break;

	case MCLASS_ATTRIB_ID_LAP: // LAP received
		AtomicIncrementVoid(&gCM->Recv.Lap);
		Status = ProcessLap(pMad, pRemoteDgrm);
		break;

	case MCLASS_ATTRIB_ID_APR: // APR received
		if (pMad->payload.APR.APStatus == APS_PATH_LOADED) {
			AtomicIncrementVoid(&gCM->Recv.AprAcc);
		} else {
			AtomicIncrementVoid(&gCM->Recv.AprRej);
		}
		Status = ProcessApr(pMad, pRemoteDgrm);
		break;

	case MCLASS_ATTRIB_ID_DREQ: // DREQ received
		AtomicIncrementVoid(&gCM->Recv.Dreq);
		Status = ProcessDreq(pMad, pRemoteDgrm);
		break;

	case MCLASS_ATTRIB_ID_DREP: // DREP received
		AtomicIncrementVoid(&gCM->Recv.Drep);
		Status = ProcessDrep(pMad, pRemoteDgrm);
		break;

	case MCLASS_ATTRIB_ID_SIDR_REQ:		// SIDR_REQ received
		AtomicIncrementVoid(&gCM->Recv.SidrReq);
		Status = ProcessSidrReq(pMad, pRemoteDgrm);
		break;

	case MCLASS_ATTRIB_ID_SIDR_RESP:		// SIDR_RESP received
		AtomicIncrementVoid(&gCM->Recv.SidrResp);
		Status = ProcessSidrResp(pMad, pRemoteDgrm);
		break;

	default:
		AtomicIncrementVoid(&gCM->Recv.Failed);
		_DBG_ERROR(("<mad 0x%p> Invalid attribute ID!!! <attr id 0x%x>\n",
					_DBG_PTR(pMad), pMad->common.AttributeID));

		break;
	}
		
	SpinLockRelease(&gCM->ListLock);
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

} // ProcessRecvCmMad()

//////////////////////////////////////////////////////////////////////////
// ProcessClassPortInfo()
// process an inbound ClassPortInfo
//
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
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL. With ListLock held
//

FSTATUS
ProcessClassPortInfo(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	FSTATUS Status=FSUCCESS;
	IBT_DGRM_ELEMENT* pDgrmElement=NULL;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessClassPortInfo);

	_DBG_INFO(("<mad 0x%p> ClassPortInfo received <method 0x%x slid 0x%x>\n",
			   _DBG_PTR(pMad), pMad->common.mr.AsReg8, pRemoteDgrm->RemoteLID));

	/* test AsReg8 so R bit included */
	if (pMad->common.mr.AsReg8 != MMTHD_GET
		&& pMad->common.mr.AsReg8 != MMTHD_SET)
	{
		_DBG_WARN(("<dgrm 0x%p> Unexpected ClassPortInfo Method 0x%x!!! discarding <dlid 0x%x>.\n", 
					_DBG_PTR(pDgrmElement), pMad->common.mr.AsReg8,
					pRemoteDgrm->RemoteLID));
		goto fail;
	}

	pDgrmElement = CmDgrmGet();
	if (!pDgrmElement)
	{
		_DBG_WARNING(("failed to send ClassPortInfo: Out of CM Dgrms!\n"));
		Status = FINSUFFICIENT_RESOURCES;
		goto fail;
	}
			
	GsiDgrmAddrCopy(pDgrmElement, pRemoteDgrm);

	FormatClassPortInfo((CM_MAD*)GsiDgrmGetSendMad(pDgrmElement),
				pMad->common.TransactionID);
	/* CM does not support traps, hence no fields to set.
	 * However, for a Set we still return a GetResp with our class info
	 */
	if (pMad->common.mr.AsReg8 == MMTHD_SET) {
		((CM_MAD*)GsiDgrmGetSendMad(pDgrmElement))->common.u.NS.Status.AsReg16 = hton16(MAD_STATUS_UNSUPPORTED_METHOD);
	}
	
	// In SendComplete(), we return the dgrm to the pool
	Status = CmDgrmSendAndRelease(pDgrmElement);
	if (Status == FSUCCESS)
	{
		AtomicIncrementVoid(&gCM->Sent.ClassPortInfoResp);
	} else {

		_DBG_WARN(("<dgrm 0x%p> DgrmSend() failed!!! Unable to send ClassPortInfo <dlid 0x%x %s>.\n", 
					_DBG_PTR(pDgrmElement),
					pRemoteDgrm->RemoteLID, _DBG_PTR(FSTATUS_MSG(Status))));

		CmDgrmPut(pDgrmElement);
	}

	// In SendComplete(), we return the dgrm to the pool
fail:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// ProcessReq()
// process an inbound REQ
//
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
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL. With ListLock held
//

FSTATUS
ProcessReq(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pCEP=NULL;
	CM_MAP_ITEM* pMapEntry = NULL;
	boolean	bFoundRemoteEndPoint=FALSE;
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessReq);

#ifdef ICS_LOGGING
	_DBG_INFO(("<mad 0x%p> REQ received <lcid 0x%x sid 0x%"PRIx64" "
					"slid 0x%x sgid 0x%"PRIx64":%"PRIx64">\n",
			   _DBG_PTR(pMad),
			   pMad->payload.REQ.LocalCommID,
			   pMad->payload.REQ.ServiceID,
			   pMad->payload.REQ.PrimaryLocalLID,
			   pMad->payload.REQ.PrimaryLocalGID.Type.Global.SubnetPrefix,
			   pMad->payload.REQ.PrimaryLocalGID.Type.Global.InterfaceID));
	
	_DBG_INFO(("<mad 0x%p> REQ received <lcid 0x%x sid 0x%"PRIx64" "
			   	"dlid 0x%x dgid 0x%"PRIx64":%"PRIx64">\n",
			   _DBG_PTR(pMad),
			   pMad->payload.REQ.LocalCommID,
			   pMad->payload.REQ.ServiceID,
			   pMad->payload.REQ.PrimaryRemoteLID,
			   pMad->payload.REQ.PrimaryRemoteGID.Type.Global.SubnetPrefix,
			   pMad->payload.REQ.PrimaryRemoteGID.Type.Global.InterfaceID));
#else
	_DBG_INFO(("<mad 0x%p> REQ received <lcid 0x%x sid 0x%"PRIx64" "
					"slid 0x%x sgid 0x%"PRIx64":%"PRIx64" "
					"dlid 0x%x dgid 0x%"PRIx64":%"PRIx64">\n",
					_DBG_PTR(pMad),
					pMad->payload.REQ.LocalCommID,
					pMad->payload.REQ.ServiceID,
					pMad->payload.REQ.PrimaryLocalLID,
					pMad->payload.REQ.PrimaryLocalGID.Type.Global.SubnetPrefix,
					pMad->payload.REQ.PrimaryLocalGID.Type.Global.InterfaceID,
					pMad->payload.REQ.PrimaryRemoteLID,
					pMad->payload.REQ.PrimaryRemoteGID.Type.Global.SubnetPrefix,
					pMad->payload.REQ.PrimaryRemoteGID.Type.Global.InterfaceID));
#endif

	if (pMad->payload.REQ.u3.s.TransportServiceType != CM_RC_TYPE
		&& pMad->payload.REQ.u3.s.TransportServiceType != CM_UC_TYPE
		&& pMad->payload.REQ.u3.s.TransportServiceType != CM_RD_TYPE
		)
	{
		_DBG_WARNING(("<mad 0x%p> Invalid transport service type!\n", _DBG_PTR(pMad)));

		SendREJ(RC_INVALID_TSTYPE, 
				CM_REJECT_REQUEST, 
				pMad->common.TransactionID,
				0, 
				pMad->payload.REQ.LocalCommID,
				pRemoteDgrm);
		goto done;
	}

	// first look for Remote Endpoint, this will catch duplicate and stale
	// requests
	pMapEntry = CM_MapGetCompare(&gCM->RemoteEndPointMap, (uintn)&pMad->payload.REQ,
							CepReqRemoteEndPointCompare);
	if (pMapEntry != CM_MapEnd(&gCM->RemoteEndPointMap))
	{
		pCEP = PARENT_STRUCT(pMapEntry, CM_CEP_OBJECT, RemoteEndPointMapEntry);
		bFoundRemoteEndPoint = TRUE;
	} else {
		// Look for the CEP object in the listen list
		pMapEntry = CM_MapGetCompare(&gCM->ListenMap, (uintn)&pMad->payload.REQ,
							CepReqAddrCompare);
		if (pMapEntry != CM_MapEnd(&gCM->ListenMap))
		{
			pCEP = PARENT_STRUCT(pMapEntry, CM_CEP_OBJECT, MapEntry);
		}
	}
	if (! pCEP)
	{
		_DBG_WARN(("<mad 0x%p> REQ dropped - no match found <lcid 0x%x sid 0x%"PRIx64">!\n", 
					_DBG_PTR(pMad), 
					pMad->payload.REQ.LocalCommID,
					pMad->payload.REQ.ServiceID));

		SendREJ(RC_INVALID_SID, 
				CM_REJECT_REQUEST, 
				pMad->common.TransactionID,
				0, 
				pMad->payload.REQ.LocalCommID, // The remote end's local comm id
				pRemoteDgrm);
		goto done;
	}
#ifdef  ICS_LOGGING
	_DBG_INFO(("<cep 0x%p> Matched REQ against <lcid 0x%x rcid 0x%x sid 0x%"PRIx64"\n",
				   _DBG_PTR(pCEP), 
				   pCEP->LocalCommID,
				   pCEP->RemoteCommID,
				   pCEP->SID));
				   
	_DBG_INFO(("<cep 0x%p> Matched REQ against "
				   "slid 0x%x sgid 0x%"PRIx64":0x%"PRIx64"\n",
				   _DBG_PTR(pCEP), 
				   pCEP->PrimaryPath.LocalLID,
				   pCEP->PrimaryPath.LocalGID.Type.Global.SubnetPrefix,
				   pCEP->PrimaryPath.LocalGID.Type.Global.InterfaceID));
		
	_DBG_INFO(("<cep 0x%p> Matched REQ against "
				   	"dlid 0x%x dgid 0x%"PRIx64":0x%"PRIx64">\n",
				   _DBG_PTR(pCEP), 
				   pCEP->PrimaryPath.RemoteLID,
				   pCEP->PrimaryPath.RemoteGID.Type.Global.SubnetPrefix,
				   pCEP->PrimaryPath.RemoteGID.Type.Global.InterfaceID));
#else
	_DBG_INFO(("<cep 0x%p> Matched REQ against <lcid 0x%x rcid 0x%x sid 0x%"PRIx64" "
					"slid 0x%x sgid 0x%"PRIx64":0x%"PRIx64" "
					"dlid 0x%x dgid 0x%"PRIx64":0x%"PRIx64">\n",
					_DBG_PTR(pCEP), 
					pCEP->LocalCommID,
					pCEP->RemoteCommID,
					pCEP->SID,
					pCEP->PrimaryPath.LocalLID,
					pCEP->PrimaryPath.LocalGID.Type.Global.SubnetPrefix,
					pCEP->PrimaryPath.LocalGID.Type.Global.InterfaceID,
					pCEP->PrimaryPath.RemoteLID,
					pCEP->PrimaryPath.RemoteGID.Type.Global.SubnetPrefix,
					pCEP->PrimaryPath.RemoteGID.Type.Global.InterfaceID));
#endif

	if (! pCEP->bPeer)
	{
		Status = ProcessReqPassive(pCEP, pMad, pRemoteDgrm);
	} else if (bFoundRemoteEndPoint) {
		// Peer Connects could have multiple similar CEPs bound
		// we must give priority to a CEP which has already
		// started working on this connection, determined by checking
		// remote End Point Address.
		Status = ProcessReqPeer(pCEP, pMad, pRemoteDgrm, NULL);
	} else {
		// For fairness, we associate this inbound REQ with the
		// earliest started REQ, hence minimizing the risk of timeouts.
		// To find all possible CEPs, we scan to either side of the CEP
		// we initially found as matching the bound address.
		// While not discussed in the spec, for local loopback connections
		// we make the association of a CEP to a remote end point in
		// CMS_REQ_SENT state to ensure consistent active/passive roles.
		// For remote connections we don't need this since CaGuid will
		// cause consistent role assignments.
		uint64 currtime_us = GetTimeStamp();
		uint64 max_elapsed = CmComputeElapsedTime(pCEP, currtime_us);
		CM_MAP_ITEM* pMapEntry2 = NULL;
		CM_CEP_OBJECT* pCEPReq = NULL;	// local CEP which sent the REQ
		ASSERT(pCEP->RemoteEndPoint.CaGUID == 0);
		if (CepLocalEndPointMatchReq(pCEP, &pMad->payload.REQ))
		{
			// the sender CEP, don't let it connect to itself
			pCEPReq = pCEP;
			pCEP = NULL;
		}
		for (pMapEntry2 = CM_MapPrev(pMapEntry);
			 pMapEntry2 != CM_MapEnd(&gCM->ListenMap);
			 pMapEntry2 = CM_MapPrev(pMapEntry2))
		{
			CM_CEP_OBJECT* pCEP2;
			pCEP2 = PARENT_STRUCT(pMapEntry2, CM_CEP_OBJECT, MapEntry);
			if (CepReqAddrCompare((uintn)pCEP2, (uintn)&pMad->payload.REQ) == 0)
			{
				uint64 elapsed;
				ASSERT(pCEP2->RemoteEndPoint.CaGUID == 0);
				if (CepLocalEndPointMatchReq(pCEP2, &pMad->payload.REQ))
				{
					// the sender CEP, don't let it connect to itself
					pCEPReq = pCEP2;
					continue;
				}
				elapsed = CmComputeElapsedTime(pCEP2, currtime_us);
				if (pCEP == NULL || elapsed > max_elapsed)
				{
					max_elapsed = elapsed;
					pCEP = pCEP2;
				}
			} else {
				break;
			}
		}
		for (pMapEntry2 = CM_MapNext(pMapEntry);
			 pMapEntry2 != CM_MapEnd(&gCM->ListenMap);
			 pMapEntry2 = CM_MapNext(pMapEntry2))
		{
			CM_CEP_OBJECT* pCEP2;
			pCEP2 = PARENT_STRUCT(pMapEntry2, CM_CEP_OBJECT, MapEntry);
			if (CepReqAddrCompare((uintn)pCEP2, (uintn)&pMad->payload.REQ) == 0)
			{
				uint64 elapsed;
				ASSERT(pCEP2->RemoteEndPoint.CaGUID == 0);
				if (CepLocalEndPointMatchReq(pCEP2, &pMad->payload.REQ))
				{
					// the sender CEP, don't let it connect to itself
					pCEPReq = pCEP2;
					continue;
				}
				elapsed = CmComputeElapsedTime(pCEP2, currtime_us);
				if (pCEP == NULL || elapsed > max_elapsed)
				{
					max_elapsed = elapsed;
					pCEP = pCEP2;
				}
			} else {
				break;
			}
		}
		if (pCEP != NULL)
			Status = ProcessReqPeer(pCEP, pMad, pRemoteDgrm, pCEPReq);
		else
			Status = FNOT_FOUND;
	}

	if (Status == FSUCCESS)
	{
		_DBG_INFO(("<mad 0x%p> REQ processed successfully <lcid 0x%x sid 0x%"PRIx64">!\n", 
						_DBG_PTR(pMad),
						pMad->payload.REQ.LocalCommID,
						pMad->payload.REQ.ServiceID));
	} else if (Status == FNOT_FOUND) {
		_DBG_WARNING(("<mad 0x%p> REQ dropped - no match found <lcid 0x%x sid 0x%"PRIx64">!\n", 
					_DBG_PTR(pMad), 
					pMad->payload.REQ.LocalCommID,
					pMad->payload.REQ.ServiceID));
	} else {
		_DBG_WARNING(("<mad 0x%p> REQ dropped - unable to process it! <lcid 0x%x sid 0x%"PRIx64" %s>\n", 
						_DBG_PTR(pMad), 
						pMad->payload.REQ.LocalCommID,
						pMad->payload.REQ.ServiceID,
						_DBG_PTR(FSTATUS_MSG(Status))));
	}
done:
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// ProcessReqPassive()
//
// This routine processes Req for a passive listening CEP
//
//
// RETURNS:
//
//	FSUCCESS
//	FERROR
//	FINVALID_STATE
//	FCM_INVALID_EVENT
//	FINSUFFICIENT_RESOURCES
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL.
//
FSTATUS
ProcessReqPassive(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_MAD*					pMad,
	IN IBT_DGRM_ELEMENT*		pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pNewCEP=NULL;
	CMM_MSG* pMsg = &pMad->payload;
	FSTATUS Status=FSUCCESS;
	uint32	worst_service_time_mult=0;
	CM_REJECTION_CODE		RejReason;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessReqPassive);

	ASSERT(! pCEP->bPeer);
	// any state other than CMS_LISTEN implies we have matched by
	// Remote EndPoint rather than by bound address
	switch (pCEP->State)
	{
		case CMS_IDLE:
			ASSERT(0);
			goto done;
			break;

			// Listening
		case CMS_LISTEN:
			break;

			// Pending Connection
		case CMS_REQ_RCVD:	// passive
		case CMS_MRA_REQ_SENT:	// passive
			// Connecting
		case CMS_REQ_SENT:	// active
		case CMS_REP_WAIT:	// active
		case CMS_REP_SENT:	// passive
		case CMS_MRA_REP_RCVD:	// passive
		case CMS_REP_RCVD:	// active
		case CMS_MRA_REP_SENT:	// active
			// connected
		case CMS_ESTABLISHED:
		case CMS_LAP_SENT:
		case CMS_MRA_LAP_SENT:
		case CMS_LAP_RCVD:
		case CMS_MRA_LAP_RCVD:
			// disconnecting
		case CMS_DREQ_SENT:
		case CMS_DREQ_RCVD:
		case CMS_SIM_DREQ_RCVD:
		case CMS_TIMEWAIT:
			if (pCEP->RemoteCommID == pMsg->REQ.LocalCommID)
			{
				// duplicate REQ
				Status = ProcessReqDup(pCEP, pMad, pRemoteDgrm);
			} else {
				// A new request instance using an existing QPN/Port 
				Status = ProcessReqStale(pCEP, pMad, pRemoteDgrm);
			}
			goto done;
			break;

			// SIDR
		case CMS_REGISTERED:
		case CMS_SIDR_REQ_SENT:
		case CMS_SIDR_REQ_RCVD:
			SendREJ(RC_INVALID_SID, CM_REJECT_REQUEST, pMad->common.TransactionID, 0, pMsg->REQ.LocalCommID, pRemoteDgrm);
			goto done;
			break;
	}

	ASSERT(pCEP->State == CMS_LISTEN);

	// If we hit this backlog cnt, start rejecting the incoming connection
	// since this listening endpoint maybe dead or very slow
	if (pCEP->PendingCount >= (pCEP->ListenBackLog?pCEP->ListenBackLog:gCmParams.MaxBackLog))
	{
		_DBG_ERROR(("<cep 0x%p> Max pending count reached!!! <sid 0x%"PRIx64" lcid 0x%x rcid 0x%x max pend cnt %d, remote CA 0x%"PRIx64">\n", 
					_DBG_PTR(pCEP), pMsg->REQ.ServiceID, pCEP->LocalCommID, pMsg->REQ.LocalCommID, pCEP->PendingCount, pMsg->REQ.LocalCaGUID));

		SendREJ(RC_NO_RESOURCES, CM_REJECT_REQUEST, pMad->common.TransactionID, 0, pMsg->REQ.LocalCommID, pRemoteDgrm);
		Status = FOVERRUN;
		goto done;
	}

	//
	// If we get here, this must be a new request.
	// The processing below follows similarly to CreateCEP() and Connect().
	//

	// Create a new CEP object to represent this request and potentially a new connection
	// TBD - we could require that TransportServiceType match that
	// of listening CEP, for now we let server potentially listen for
	// multiple service types on 1 CEP
	pNewCEP = CreateCEP((CM_CEP_TYPE)pMsg->REQ.u3.s.TransportServiceType, NULL, NULL);
	if (!pNewCEP)
	{
		_DBG_WARNING(("CreateCEP() failed!\n"));

		// Send a reject
		SendREJ(RC_NO_RESOURCES, CM_REJECT_REQUEST, pMad->common.TransactionID, 0, pMsg->REQ.LocalCommID, pRemoteDgrm);

		Status = FINSUFFICIENT_RESOURCES;
		goto done;
	}

	(void)iba_get_ca_for_port(pRemoteDgrm->PortGuid, &pNewCEP->LocalEndPoint.CaGUID);
	Status = SaveReqToPassiveCEP(pNewCEP, &pMsg->REQ, pNewCEP->LocalEndPoint.CaGUID, pRemoteDgrm->PortGuid, &RejReason);
	if (Status != FSUCCESS)
	{
		_DBG_WARNING(("Gid/PKey validation() failed!\n"));

		// Send a reject
		SendREJ(RejReason, CM_REJECT_REQUEST, pMad->common.TransactionID, 0, pMsg->REQ.LocalCommID, pRemoteDgrm);
		DestroyCEP(pNewCEP);
		goto done;
	}

	// Move from IDLE to PENDING
	CepAddToPendingList(pCEP, pNewCEP);
	CepSetState(pNewCEP, CMS_REQ_RCVD);

	// Set the mode to passive since this is the server-side
	pNewCEP->Mode = CM_MODE_PASSIVE;

	CmDgrmCopyAddrAndMad(pNewCEP->pDgrmElement, pMad, pRemoteDgrm);

	// Set various persistence info for this obj
	AssignLocalCommID(pNewCEP);
	pNewCEP->TransactionID = pMad->common.TransactionID; // active sets TID
	pNewCEP->RemoteCommID = pMsg->REQ.LocalCommID;

	// Save the local endpoint addr based on the connect request.
	// We will bind this object to the QPN when user Accept().
	pNewCEP->SID = pMsg->REQ.ServiceID;
	pNewCEP->LocalEndPoint.EECN = pMsg->REQ.u3.s.RemoteEECN;

	// Save the remote endpoint addr based on the connect request
	pNewCEP->RemoteEndPoint.CaGUID = pMsg->REQ.LocalCaGUID;
	pNewCEP->RemoteEndPoint.QPN = pMsg->REQ.u1.s.LocalQPN;
	pNewCEP->RemoteEndPoint.EECN = pMsg->REQ.u2.s.LocalEECN;

	// RemoteEndPoint is unique
	CM_MapInsert(&gCM->RemoteEndPointMap, (uintn)pNewCEP, &pNewCEP->RemoteEndPointMapEntry, "REMOTE_END_POINT_LIST", pNewCEP);

	// inherit turnaround time for use in ServiceTime for MRA retries
	// will update again when CmAccept to pick up any adjusted values
	pNewCEP->turnaround_time_us = pCEP->turnaround_time_us;

	// No user callback info
	pNewCEP->pfnUserCallback = NULL;
	pNewCEP->UserContext = NULL;

	// compare RemoteCMTimeout to our Worst expected turnaround
	// if necessary, output an MRA to be safe
	worst_service_time_mult = CepWorstServiceTimeMult(pNewCEP);
	if (pMsg->REQ.u3.s.RemoteCMTimeout < worst_service_time_mult) {
		SendMRA(CM_MRA_REQUEST, worst_service_time_mult,
			pNewCEP->TransactionID, pNewCEP->LocalCommID,
			pNewCEP->RemoteCommID, pRemoteDgrm);
		CepSetState(pNewCEP, CMS_MRA_REQ_SENT);
	}

	CmStartElapsedTime(pNewCEP);	// for turnaround computation
	// Signal the listening obj if this is the first on the pending list
	// otherwise, accept() or reject() will handle this
	if (pCEP->PendingCount == 1)
	{
		SetNotification(pCEP, CME_RCVD_REQ); //EventSet(pCEP, CME_RCVD_REQ);
	}
	// At this point, the user accept(), reject() or a timeout occurs

done:
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

} // ProcessReqPassive()

//////////////////////////////////////////////////////////////////////////
// ProcessReqPeer()
//
// Process inbound REQ for a Peer Connect CEP
//
//
// RETURNS:
//
//	FSUCCESS 
//	FERROR
//	FINVALID_STATE
//	FCM_INVALID_EVENT
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL.
//
FSTATUS
ProcessReqPeer(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_MAD*					pMad,
	IN IBT_DGRM_ELEMENT*		pRemoteDgrm,
	IN CM_CEP_OBJECT*			pCEPReq
	)
{
	CMM_MSG* pMsg = &pMad->payload;
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessReqPeer);

	if (pCEP->State != CMS_REQ_SENT)
	{
		// we must have matched on Remote Address in ProcessReq
		ASSERT(CepRemoteEndPointMatchReq(pCEP, &pMsg->REQ));
		if (pCEP->RemoteCommID == pMsg->REQ.LocalCommID)
		{
			Status = ProcessReqDup(pCEP, pMad, pRemoteDgrm);
		} else {
			// A new request instance using an existing QPN/Port ?
			Status = ProcessReqStale(pCEP, pMad, pRemoteDgrm);
		}
		goto done;
	}

	//
	// New peer connect request - determine active or passive role
	//
	_DBG_INFO(("<cep 0x%p> *** New peer request *** <lcid 0x%x rcid 0x%x>.\n",
					_DBG_PTR(pCEP), pCEP->LocalCommID, pMsg->REQ.LocalCommID));

	// There should NOT be anything on the PENDING list
	if (!DListIsEmpty(&pCEP->PendingList))
	{
		Status = FERROR;
		goto done;
	}
			
	// Set this end to active or passive
	if (pCEP->LocalEndPoint.CaGUID < pMsg->REQ.LocalCaGUID)
	{
		// This endpoint takes the passive role
		pCEP->Mode = CM_MODE_PASSIVE;
	} else if (pCEP->LocalEndPoint.CaGUID > pMsg->REQ.LocalCaGUID) {
		// This endpoint takes the active role
		pCEP->Mode = CM_MODE_ACTIVE;
	} else {
		if (pCEP->LocalEndPoint.QPN < pMsg->REQ.u1.s.LocalQPN)
		{
			// This endpoint takes the passive role
			pCEP->Mode = CM_MODE_PASSIVE;
		} else if (pCEP->LocalEndPoint.QPN > pMsg->REQ.u1.s.LocalQPN) {
			// This endpoint takes the active role
			pCEP->Mode = CM_MODE_ACTIVE;
		} else {
			Status = FERROR;
			goto done;
		}
	}

	// active sets paths and timeouts
	// validate GID/PKey in REQ
	if (pCEP->Mode == CM_MODE_PASSIVE)
	{
		CM_REJECTION_CODE		RejReason;

		Status = SaveReqToPassiveCEP(pCEP, &pMsg->REQ, pCEP->LocalEndPoint.CaGUID, pRemoteDgrm->PortGuid, &RejReason);
		if (Status != FSUCCESS)
		{
			_DBG_WARNING(("Gid/PKey validation() failed!\n"));

			// Send a reject
			SendREJ(RejReason, CM_REJECT_REQUEST, pMad->common.TransactionID, 0, pMsg->REQ.LocalCommID, pRemoteDgrm);
			// restore CEP, still waiting for valid Peer REQ
			pCEP->Mode = CM_MODE_ACTIVE;
			goto done;
		}
	}

	// for a local loopback connection we must ensure consistent
	// interpretation of the the roles and consistent association
	// of the end points, so we set the Remote EndPoint for both CEPs
	if (pCEPReq != NULL)
	{
		// RemoteEndPoints must be unique since we know the local's
		// are unique and we are a loopback
		ASSERT(pCEP != pCEPReq);
		if (pCEPReq->RemoteEndPoint.CaGUID == 0) 
		{
			ASSERT(pCEPReq->State == CMS_REQ_SENT || pCEPReq->State == CMS_REP_WAIT);
			CM_MapRemoveEntry(&gCM->ListenMap, &pCEPReq->MapEntry, LISTEN_LIST, pCEPReq);
			pCEPReq->RemoteEndPoint.CaGUID = pCEP->LocalEndPoint.CaGUID;
			pCEPReq->RemoteEndPoint.QPN = pCEP->LocalEndPoint.QPN;
			pCEPReq->RemoteEndPoint.EECN = pCEP->LocalEndPoint.EECN;
			CM_MapInsert(&gCM->RemoteEndPointMap, (uintn)pCEPReq, &pCEPReq->RemoteEndPointMapEntry, "REMOTE_END_POINT_LIST", pCEPReq);
		}

		if (pCEP->RemoteEndPoint.CaGUID == 0) 
		{
			CM_MapRemoveEntry(&gCM->ListenMap, &pCEP->MapEntry, LISTEN_LIST, pCEP);
			pCEP->RemoteEndPoint.CaGUID = pMsg->REQ.LocalCaGUID;
			pCEP->RemoteEndPoint.QPN = pMsg->REQ.u1.s.LocalQPN; 
			pCEP->RemoteEndPoint.EECN = pMsg->REQ.u2.s.LocalEECN; 
			CM_MapInsert(&gCM->RemoteEndPointMap, (uintn)pCEP, &pCEP->RemoteEndPointMapEntry, "REMOTE_END_POINT_LIST", pCEP);
		}
	}

	if (pCEP->Mode == CM_MODE_PASSIVE)
	{
		uint32 worst_service_time_mult;

		_DBG_INFO(("<cep 0x%p> *** This peer endpoint takes the passive role *** <lcid 0x%x rcid 0x%x>.\n",
					_DBG_PTR(pCEP), pCEP->LocalCommID, pMsg->REQ.LocalCommID));

		pCEP->TransactionID = pMad->common.TransactionID; // active sets TID

		// Cancel the timer on REQ
		CmTimerStop(pCEP, REQ_TIMER);

		CepAddToPendingList(pCEP, pCEP);	// Peer is linked to self
		CepSetState(pCEP, CMS_REQ_RCVD);

		// REQ -->
		// REQ <--
		Status = PrepareCepDgrm(pCEP);
		if (Status != FSUCCESS)
			goto fail;

		// Save the request msg
		CmDgrmCopyAddrAndMad(pCEP->pDgrmElement, pMad, pRemoteDgrm);

		// Save the rest of the request info to this endpoint
		pCEP->RemoteCommID = pMsg->REQ.LocalCommID;
		if (pCEP->RemoteEndPoint.CaGUID == 0)
		{
			// we looked up by RemoteEndPoint in ProcessReq
			// and to get here we must not have found it as a dup
			CM_MapRemoveEntry(&gCM->ListenMap, &pCEP->MapEntry, LISTEN_LIST, pCEP);
			pCEP->RemoteEndPoint.CaGUID = pMsg->REQ.LocalCaGUID;
			pCEP->RemoteEndPoint.QPN = pMsg->REQ.u1.s.LocalQPN; 
			pCEP->RemoteEndPoint.EECN = pMsg->REQ.u2.s.LocalEECN; 
			CM_MapInsert(&gCM->RemoteEndPointMap, (uintn)pCEP, &pCEP->RemoteEndPointMapEntry, "REMOTE_END_POINT_LIST", pCEP);
		}

		// compare RemoteCMTimeout to our typical turnaround
		// if necessary, output an MRA to be safe
		worst_service_time_mult = CepWorstServiceTimeMult(pCEP);
		if (pMsg->REQ.u3.s.RemoteCMTimeout < worst_service_time_mult) {
			SendMRA(CM_MRA_REQUEST, worst_service_time_mult,
				pCEP->TransactionID, pCEP->LocalCommID,
				pCEP->RemoteCommID, pRemoteDgrm);
			CepSetState(pCEP, CMS_MRA_REQ_SENT);
		}

		CmStartElapsedTime(pCEP);	// for turnaround computation
		// Update the event flag of the listening obj
		SetNotification(pCEP, CME_RCVD_REQ);//EventSet(pCEP, CME_RCVD_REQ);

		// At this point , the user accept(), reject() or a timeout occurs.
	} else {
		_DBG_INFO(("<cep 0x%p> *** This peer endpoint takes the active role *** <lcid 0x%x rcid 0x%x>,\n",
					_DBG_PTR(pCEP), pCEP->LocalCommID, pMsg->REQ.LocalCommID));

		// When the active peer receives a REQ, we resend the REQ to the
		// passive peer to avoid waiting for the timeout to resend the REQ.
		// This reduces the connection establishment time when we bring up
		// the active peer then passive peer.
		CmTimerStop(pCEP, REQ_TIMER);
		ResendREQ(pCEP);
	}

done:
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

fail:
	_DBG_WARNING(("<mad 0x%p> REQ dropped - unable to process it! <lcid 0x%x rcid 0x%x %s>\n",
					_DBG_PTR(pMad), pCEP->LocalCommID, pMsg->REQ.LocalCommID,
					_DBG_PTR(FSTATUS_MSG(Status))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

} // ProcessReqPeer()

// Save REQ Path and timers into CEP
FSTATUS
SaveReqToPassiveCEP(
	IN CM_CEP_OBJECT*			pCEP,
	IN CMM_REQ*					pREQ,
	IN EUI64					CaGUID,
	IN EUI64					PortGuid,	// port which received REQ
	OUT CM_REJECTION_CODE		*RejReason	// only returned on failure
	)
{
	FSTATUS Status;
	IB_PORT_ATTRIBUTES *pPortAttr = NULL;
	EUI64 PrimaryLocalPortGuid;
	uint8 PrimaryLocalGidIndex;
	uint8 PrimaryLocalPathBits;
	uint16 PrimaryPkeyIndex = 0;
	EUI64 AlternateLocalPortGuid = 0;
	uint8 AlternateLocalGidIndex = 0;
	uint8 AlternateLocalPathBits = 0;
	uint16 AlternatePkeyIndex = 0;
	IB_CA_ATTRIBUTES *pCaAttr = NULL;

	// first validate REQ, don't change CEP til sure its valid
	Status = iba_find_ca_gid2(&pREQ->PrimaryRemoteGID, CaGUID,
					&PrimaryLocalPortGuid, &PrimaryLocalGidIndex, &pPortAttr,
					&pCaAttr);
	if (Status != FSUCCESS && gCmParams.DisableValidation)
	{
		pCaAttr = (IB_CA_ATTRIBUTES*)MemoryAllocate(sizeof(*pCaAttr), FALSE, CM_MEM_TAG);
		Status = iba_query_port_by_guid_alloc(PortGuid, &pPortAttr);
		if (pCaAttr == NULL || Status != FSUCCESS) {
			*RejReason = RC_NO_RESOURCES;
			Status = FINSUFFICIENT_MEMORY;
			goto done;
		}
		pCaAttr->PortAttributesListSize = 0;
		pCaAttr->PortAttributesList = NULL;
		Status = iba_query_ca_by_guid(CaGUID, pCaAttr);
		if (Status == FSUCCESS)
		{
			PrimaryLocalPortGuid = PortGuid;
			PrimaryLocalGidIndex = 0;
		}
	}
	if (Status != FSUCCESS || PrimaryLocalPortGuid != PortGuid)
	{
		*RejReason = RC_PRIMARY_DGID_REJ;
		Status = FINVALID_PARAMETER;
		goto done;
	}
	if (! ValidatePKey(pPortAttr, pREQ->PartitionKey, &PrimaryPkeyIndex))
	{
		if (! gCmParams.DisableValidation)
		{
			*RejReason = RC_UNSUPPORTED_REQ;	// TBD
			Status = FINVALID_PARAMETER;
			goto done;
		}
		Status = FSUCCESS;
		PrimaryPkeyIndex = 0;	// default
	}
	if (pPortAttr)
	{
		PrimaryLocalPathBits = LidToPathBits(pREQ->PrimaryRemoteLID, pPortAttr->Address.LMC);
		MemoryDeallocate(pPortAttr);
	} else {
		PrimaryLocalPathBits = 0;
	}
	pPortAttr = NULL;
	if (pREQ->AlternateRemoteLID != 0)
	{
		Status = iba_find_ca_gid(&pREQ->AlternateRemoteGID, CaGUID,
					&AlternateLocalPortGuid, &AlternateLocalGidIndex,
					&pPortAttr, NULL);
		// DisableValidation does not apply, we need to know
		// the alternate port Guid so we can failover the CM messages
		// we cannot reliably go off the RemoteLID because ports
		// could have same LID if they are on different fabrics
		if (Status != FSUCCESS)
		{
			*RejReason = RC_ALTERNATE_DGID;
			Status = FINVALID_PARAMETER;
			goto done;
		}
		if (! ValidatePKey(pPortAttr, pREQ->PartitionKey, &AlternatePkeyIndex))
		{
			if (! gCmParams.DisableValidation)
			{
				*RejReason = RC_UNSUPPORTED_REQ;	// TBD
				Status = FINVALID_PARAMETER;
				goto done;
			}
			AlternatePkeyIndex = 0;	// default
			Status = FSUCCESS;
		}
		if (pPortAttr)
		{
			AlternateLocalPathBits = LidToPathBits(pREQ->AlternateRemoteLID, pPortAttr->Address.LMC);
			MemoryDeallocate(pPortAttr);
		} else {
			AlternateLocalPathBits = 0;
		}
		pPortAttr = NULL;
	}

	// Primary Path
	pCEP->PrimaryPath.LocalPortGuid = PrimaryLocalPortGuid;
	pCEP->PrimaryPath.LocalGID =  pREQ->PrimaryRemoteGID;
	pCEP->PrimaryPath.RemoteGID = pREQ->PrimaryLocalGID;
	pCEP->PrimaryPath.LocalLID =  pREQ->PrimaryRemoteLID;
	pCEP->PrimaryPath.RemoteLID = pREQ->PrimaryLocalLID;
	pCEP->PrimaryPath.FlowLabel = pREQ->u5.s.PrimaryFlowLabel;
	pCEP->PrimaryPath.StaticRate = pREQ->u5.s.PrimaryPacketRate;
	pCEP->PrimaryPath.TClass = pREQ->PrimaryTrafficClass;
	pCEP->PrimaryPath.SL = pREQ->PrimarySL;
	pCEP->PrimaryPath.bSubnetLocal = pREQ->PrimarySubnetLocal;
	pCEP->PrimaryPath.AckTimeout = pREQ->PrimaryLocalAckTimeout;
	pCEP->PrimaryPath.LocalAckTimeout = pREQ->PrimaryLocalAckTimeout;
	pCEP->PrimaryPath.HopLimit = pREQ->PrimaryHopLimit;
	pCEP->PrimaryPath.LocalGidIndex = PrimaryLocalGidIndex;
	pCEP->PrimaryPath.LocalPathBits = PrimaryLocalPathBits;
	pCEP->PrimaryPath.PkeyIndex = PrimaryPkeyIndex;

	// Alternate Path
	pCEP->AlternatePath.LocalPortGuid = AlternateLocalPortGuid;
	pCEP->AlternatePath.LocalGID =  pREQ->AlternateRemoteGID;
	pCEP->AlternatePath.RemoteGID = pREQ->AlternateLocalGID;
	pCEP->AlternatePath.LocalLID =  pREQ->AlternateRemoteLID;
	pCEP->AlternatePath.RemoteLID = pREQ->AlternateLocalLID;
	pCEP->AlternatePath.FlowLabel = pREQ->u6.s.AlternateFlowLabel;
	pCEP->AlternatePath.StaticRate = pREQ->u6.s.AlternatePacketRate;
	pCEP->AlternatePath.TClass = pREQ->AlternateTrafficClass;
	pCEP->AlternatePath.SL = pREQ->AlternateSL;
	pCEP->AlternatePath.bSubnetLocal = pREQ->AlternateSubnetLocal;
	pCEP->AlternatePath.AckTimeout = pREQ->AlternateLocalAckTimeout;
	pCEP->AlternatePath.LocalAckTimeout = pREQ->AlternateLocalAckTimeout;
	pCEP->AlternatePath.HopLimit = pREQ->AlternateHopLimit;
	pCEP->AlternatePath.LocalGidIndex = AlternateLocalGidIndex;
	pCEP->AlternatePath.LocalPathBits = AlternateLocalPathBits;
	pCEP->AlternatePath.PkeyIndex = AlternatePkeyIndex;

	// negotiate down before we present REQ to listener
	if (pREQ->u3.s.TransportServiceType == CM_RC_TYPE) {
		pCEP->LocalInitiatorDepth = MIN(pREQ->u1.s.OfferedResponderResources,
								pCaAttr->MaxQPInitiatorDepth);
		pCEP->LocalResponderResources = MIN(pREQ->u2.s.OfferedInitiatorDepth,
								pCaAttr->MaxQPResponderResources);
	} else if (pREQ->u3.s.TransportServiceType == CM_RD_TYPE) {
		pCEP->LocalInitiatorDepth = MIN(pREQ->u1.s.OfferedResponderResources,
								pCaAttr->MaxEECInitiatorDepth);
		pCEP->LocalResponderResources = MIN(pREQ->u2.s.OfferedInitiatorDepth,
								pCaAttr->MaxEECResponderResources);
	} else {
		pCEP->LocalInitiatorDepth = 0;
		pCEP->LocalResponderResources = 0;
	}
	// Ack timeout is calculated as round-trip
	// This computation will be rounded up by CA Ack Delay/2
	if (pREQ->PrimaryLocalAckTimeout >= 1)
	{
		// -1 for a log2 value is the same as /2
		pCEP->PktLifeTime = (pREQ->PrimaryLocalAckTimeout-1);
	} else {
		pCEP->PktLifeTime = 0;
	}

	pCEP->TargetAckDelay = pCaAttr->LocalCaAckDelay;// application could round up
	// save parameters to help us later
	pCEP->LocalRnrRetryCount = pREQ->RnRRetryCount;
	pCEP->LocalRetryCount = pREQ->u4.s.RetryCount;
	pCEP->LocalSendPSN = pREQ->u4.s.StartingPSN;

	// timewait value.  This will be updated when we send the REP
	// If we get a REJ/RC_STALE_CONN, we will use this value for our Timewait
	pCEP->Timewait = pREQ->PrimaryLocalAckTimeout;

	// If we are unable to process the REQ and send the REP before this timeout,
	// we will send an MRA to prevent the other end from resending the REQ.
	pCEP->RemoteCMTimeout = pREQ->u3.s.RemoteCMTimeout;

	// This timeout is used to set the RTU timeout when REP is sent out.
	// If the time expires i.e. we did not received the RTU, we need to resend
	// the REP.
	pCEP->LocalCMTimeout = pREQ->u4.s.LocalCMTimeout;

	// How many times to retry the REP. We put a limit of CM_REP_RETRY for the server-side
	// because the server should not be tied down doing retries.
	pCEP->MaxCMRetries = (pREQ->MaxCMRetries > gCmParams.MaxRepRetry)?(gCmParams.MaxRepRetry):(pREQ->MaxCMRetries);

	pCEP->Mtu = pREQ->PathMTU;
	pCEP->PartitionKey = pREQ->PartitionKey;
done:
	if (pCaAttr)
		MemoryDeallocate(pCaAttr);
	if (pPortAttr)
		MemoryDeallocate(pPortAttr);
	return Status;
}

// Process a Duplicate REQ
FSTATUS
ProcessReqDup(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_MAD*					pMad,
	IN IBT_DGRM_ELEMENT*		pRemoteDgrm
	)
{
	CMM_MSG* pMsg = &pMad->payload;
	uint32 worst_service_time_mult;
	uint64 new_turnaround;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessReqDup);

	_DBG_WARNING(("<cep 0x%p> Duplicate REQ detected in %s <lcid 0x%x rcid 0x%x>!\n",
				_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State)),
				pCEP->LocalCommID, pMsg->REQ.LocalCommID));

	// TBD - CMS_REQ_RCVD - could MRA it here, instead we let CmAccept REP it
	switch (pCEP->State)
	{
	case CMS_REP_SENT:
		CmTimerStop(pCEP, REP_TIMER);
		ResendREP(pCEP);
		break;
	case CMS_MRA_REQ_SENT:
		// Update the turnaround time
		new_turnaround = UpdateTurnaroundTime(gCM->turnaround_time_us,  2*CmGetElapsedTime(pCEP));
		if (new_turnaround > gCM->turnaround_time_us)
		{
			gCM->turnaround_time_us = new_turnaround;
		}
		worst_service_time_mult = CepWorstServiceTimeMult(pCEP);
		SendMRA(CM_MRA_REQUEST,
			MAX(worst_service_time_mult, pMsg->REQ.u3.s.RemoteCMTimeout),
			pCEP->TransactionID, pCEP->LocalCommID,
			pCEP->RemoteCommID, pRemoteDgrm);
		break;
	case CMS_REQ_RCVD:
		new_turnaround = UpdateTurnaroundTime(gCM->turnaround_time_us,  2*CmGetElapsedTime(pCEP));
		if (new_turnaround > gCM->turnaround_time_us)
		{
			gCM->turnaround_time_us = new_turnaround;
		}
		worst_service_time_mult = CepWorstServiceTimeMult(pCEP);
		if (pMsg->REQ.u3.s.RemoteCMTimeout < worst_service_time_mult) {
			SendMRA(CM_MRA_REQUEST, worst_service_time_mult,
				pCEP->TransactionID, pCEP->LocalCommID,
				pCEP->RemoteCommID, pRemoteDgrm);
			CepSetState(pCEP, CMS_MRA_REQ_SENT);
		}
		break;
	default:
		break;
	}

	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,FSUCCESS);
	return FSUCCESS;
}

// Process a Stale REQ
FSTATUS
ProcessReqStale(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_MAD*					pMad,
	IN IBT_DGRM_ELEMENT*		pRemoteDgrm
	)
{
	FSTATUS Status = FSUCCESS;
	CMM_MSG* pMsg = &pMad->payload;
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessReqStale);

	// A new request instance using an existing QPN/Port 
	_DBG_WARNING(("<cep 0x%p> Stale REQ detected, send reject < lcid 0x%x rcid 0x%x %s>!\n",
			_DBG_PTR(pCEP), pCEP->LocalCommID, pMsg->REQ.LocalCommID,
			_DBG_PTR(CmGetStateString(pCEP->State))));

	// TBD - should send REJ for all states?
	switch (pCEP->State)
	{
		case CMS_REQ_RCVD:	// passive
		case CMS_MRA_REQ_SENT:	// passive
			// Connecting
		case CMS_REQ_SENT:	// active
		case CMS_REP_WAIT:	// active
		case CMS_REP_SENT:	// passive
		case CMS_MRA_REP_RCVD:	// passive
		case CMS_REP_RCVD:	// active
		case CMS_MRA_REP_SENT:	// active
		case CMS_ESTABLISHED:	// active
		case CMS_LAP_SENT:	// active
		case CMS_MRA_LAP_RCVD:	// active
		case CMS_LAP_RCVD:	// passive
		case CMS_MRA_LAP_SENT:	// passive
			// Send a reject
			SendREJ(RC_STALE_CONN, CM_REJECT_REQUEST, pMad->common.TransactionID, 0, pMsg->REQ.LocalCommID, pRemoteDgrm);
			break;
		default:
			break;
	}

	// TBD Disconnect local connection for other states?
	switch (pCEP->State)
	{
		case CMS_ESTABLISHED:	// active
		case CMS_LAP_SENT:	// active
		case CMS_MRA_LAP_RCVD:	// active
		case CMS_LAP_RCVD:	// passive
		case CMS_MRA_LAP_SENT:	// passive
			// Additionally, we must send a DREQ to disconnect the previous
			// connection instance at the remote endpoint.
			SendDREQ(pCEP->RemoteEndPoint.QPN, pMad->common.TransactionID,
						pCEP->LocalCommID, pCEP->RemoteCommID, pRemoteDgrm);

			// Notify local user existing connection is disconnected
			// Fake out the new REQ as a DREQ for the connection
			MemoryClear(pMad, sizeof(CM_MAD));
			pMad->payload.DREQ.LocalCommID = pCEP->RemoteCommID;
			pMad->payload.DREQ.RemoteCommID = pCEP->LocalCommID;
			pMad->payload.DREQ.u.s.RemoteQPNorEECN = pCEP->LocalEndPoint.QPN;
			Status = ProcessDreqCep(pCEP, pMad, pRemoteDgrm);
// TBD we should provide a DREQ to our local app, but be in a DREQ_SENT state
// waiting for the DREP and retry the DREQ until we timeout or get a DREP
			break;
		default:
			break;
	}
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// ProcessRep()
// process an inbound REP
//
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
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL. With ListLock held
//

FSTATUS
ProcessRep(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pCEP=NULL;
	FSTATUS Status=FSUCCESS;
	uint32 worst_service_time_mult;
	uint64 new_turnaround;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessRep);

	_DBG_INFO(("<mad 0x%p> REP received <lcid 0x%x rcid 0x%x>\n",					
					_DBG_PTR(pMad),
					pMad->payload.REP.LocalCommID,
					pMad->payload.REP.RemoteCommID));

	pCEP = FindLocalCommID(pMad->payload.REP.RemoteCommID);
	if (! pCEP) {
		goto nomatch;
	}
	// TODO: Check the mad's addr with our endpoint's addr in addition to the comm ID, also validate RemoteCommID
	// these should generate assorted REJ conditions if it doesn't match
	switch (pCEP->State)
	{
		case CMS_REQ_SENT:
		case CMS_REP_WAIT:
			// Received the reply to our connect request
			// notify user so they can CmAccept to cause an RTU
			Status = PrepareCepDgrm(pCEP);
			if (Status != FSUCCESS)
				goto fail;
			
			if (pCEP->RemoteEndPoint.CaGUID == 0)
			{
				pCEP->RemoteEndPoint.CaGUID = pMad->payload.REP.LocalCaGUID;
				pCEP->RemoteEndPoint.QPN = pMad->payload.REP.u1.s.LocalQPN; 
				pCEP->RemoteEndPoint.EECN = pMad->payload.REP.u2.s.LocalEECN; 

				if (! CM_MapTryInsert(&gCM->RemoteEndPointMap, (uintn)pCEP, &pCEP->RemoteEndPointMapEntry, "REMOTE_END_POINT_LIST", pCEP))
				{
					// unexpected remote end point, the new REP is stale
					pCEP->RemoteEndPoint.CaGUID =  0;
					pCEP->RemoteEndPoint.QPN = 0;
					pCEP->RemoteEndPoint.EECN = 0;
					goto stale;
				}
				if (pCEP->bPeer)
				{
					CM_MapRemoveEntry(&gCM->ListenMap, &pCEP->MapEntry, LISTEN_LIST, pCEP);
				}
			} else {
				ASSERT(pCEP->bPeer);
				// unexpected remote end point, the new REP is stale
				if (! CepRemoteEndPointMatchRep(pCEP, &pMad->payload.REP))
				{
					goto stale;
				}
			}

			CmTimerStop(pCEP, REQ_TIMER);	 // Cancel the timer on REQ
			pCEP->RemoteCommID = pMad->payload.REP.LocalCommID;
			if (pCEP->AlternatePath.LocalLID != 0)
			{
				if (pMad->payload.REP.FailoverAccepted != CM_REP_FO_ACCEPTED)
				{
					MemoryClear(&pCEP->AlternatePath,
									sizeof(pCEP->AlternatePath));
				}
				pCEP->bFailoverSupported &= IsFailoverSupported(
											pMad->payload.REP.FailoverAccepted);
			}

			// save arbitrated RDMA Read parameters
			// previously we saved the values we sent, only allow decreases
			// BUGBUG, technically we should REJ the REP if values are >
			if (pMad->payload.REP.ArbInitiatorDepth < pCEP->LocalResponderResources)
				pCEP->LocalResponderResources = pMad->payload.REP.ArbInitiatorDepth;
			if (pMad->payload.REP.ArbResponderResources < pCEP->LocalInitiatorDepth)
				pCEP->LocalInitiatorDepth = pMad->payload.REP.ArbResponderResources;

			// previously we estimated TargetAckDelay as LocalCaAckDelay
			// now update CM timers to account for real TargetAckDelay
			// some applications don't set TargetAckDelay properly
			// so be conservative and only increase timeouts
			if (pMad->payload.REP.TargetAckDelay > pCEP->TargetAckDelay)
			{
				uint64 delta = TimeoutMultToTimeInUsec(pMad->payload.REP.TargetAckDelay)
								- TimeoutMultToTimeInUsec(pCEP->TargetAckDelay);
				// technically LocalCMTimeout and RemoteCMTimeout do not
				// have a target ack delay component, but to be safe we
				// increase the timeout since CA processing overhead
				// will be part of the service time
				pCEP->LocalCMTimeout = TimeoutTimeToMult(
							TimeoutMultToTimeInUsec(pCEP->LocalCMTimeout)
							+ delta);
				pCEP->RemoteCMTimeout = TimeoutTimeToMult(
							TimeoutMultToTimeInUsec(pCEP->RemoteCMTimeout)
							+ delta);
				pCEP->Timewait = TimeoutTimeToMult(
							TimeoutMultToTimeInUsec(pCEP->Timewait)
							+ delta);
				pCEP->TargetAckDelay = pMad->payload.REP.TargetAckDelay;
			}

			// save parameters for later
			pCEP->LocalSendPSN = pMad->payload.REP.u3.s.StartingPSN;
			pCEP->LocalRnrRetryCount = pMad->payload.REP.RnRRetryCount;
			// when we sent REQ we saved PktLifeTime in LocalAckTimeout
			// now factor in TargetAckDelay to get real timeout
			pCEP->PrimaryPath.LocalAckTimeout = ComputeAckTimeout(
					pCEP->PrimaryPath.LocalAckTimeout,
					pCEP->TargetAckDelay);
			if (pCEP->AlternatePath.LocalLID != 0)
			{
				pCEP->AlternatePath.LocalAckTimeout = ComputeAckTimeout(
					pCEP->AlternatePath.LocalAckTimeout,
					pCEP->TargetAckDelay);
			}

			// Save the reply msg
			CmDgrmCopyAddrAndMad(pCEP->pDgrmElement, pMad, pRemoteDgrm);

			CepSetState(pCEP, CMS_REP_RCVD);

			CmStartElapsedTime(pCEP);	// for turnaround computation

			// Update the event flag
			SetNotification(pCEP, CME_RCVD_REP); //EventSet(pCEP, CME_RCVD_REP);
			break;
		case CMS_REQ_RCVD:
		case CMS_MRA_REQ_SENT:
		case CMS_REP_SENT:
		case CMS_MRA_REP_RCVD:
			goto discard;
			break;
		case CMS_MRA_REP_SENT:
			new_turnaround = UpdateTurnaroundTime(gCM->turnaround_time_us,  2*CmGetElapsedTime(pCEP));
			if (new_turnaround > gCM->turnaround_time_us)
			{
				gCM->turnaround_time_us = new_turnaround;
			}
			worst_service_time_mult = CepWorstServiceTimeMult(pCEP);
			SendMRA(CM_MRA_REPLY,
				MAX(worst_service_time_mult, pCEP->RemoteCMTimeout),
				pCEP->TransactionID, pCEP->LocalCommID,
				pCEP->RemoteCommID, pRemoteDgrm);
			break;
		case CMS_REP_RCVD:
			new_turnaround = UpdateTurnaroundTime(gCM->turnaround_time_us,  2*CmGetElapsedTime(pCEP));
			if (new_turnaround > gCM->turnaround_time_us)
			{
				gCM->turnaround_time_us = new_turnaround;
			}
			worst_service_time_mult = CepWorstServiceTimeMult(pCEP);
			// just in case remote end miscomputed RemoteCMTimeout
			// we always send MRA in this case
			SendMRA(CM_MRA_REPLY,
				MAX(worst_service_time_mult, pCEP->RemoteCMTimeout),
				pCEP->TransactionID, pCEP->LocalCommID,
				pCEP->RemoteCommID, pRemoteDgrm);
				CepSetState(pCEP, CMS_MRA_REP_SENT);
			break;
		case CMS_ESTABLISHED:
		case CMS_LAP_SENT:
		case CMS_MRA_LAP_RCVD:
			// Resend the RTU
			if (pCEP->bLapSent)
			{
				// pCEP->pDgrmElement is not RTU, send an empty RTU
				SendRTU(pCEP->TransactionID, pCEP->LocalCommID, 
						pCEP->RemoteCommID, pRemoteDgrm);
				break;
			}
			// Dgrm will be an RTU
			// If dgrm is still in use (i.e. send pending), skip the retry
			// because pDgrmElement is still on SendQ
			if (!CmDgrmIsInUse(pCEP->pDgrmElement))
			{
				ASSERT(GsiDgrmGetSendMad(pCEP->pDgrmElement)->common.AttributeID  == MCLASS_ATTRIB_ID_RTU);
				_DBG_INFO(("<cep 0x%p> RTU resend <lcid 0x%x rcid 0x%x>\n", _DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RemoteCommID));
				Status = CmDgrmSend(pCEP->pDgrmElement);
				if (FSUCCESS != Status)
				{
					// fall through and let timer retry the send later
					_DBG_WARN(("<cep 0x%p> DgrmSend() failed for RTU!!! <%s>\n",
							_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
					Status = FSUCCESS;
				} else {
					AtomicIncrementVoid(&gCM->Sent.Rtu);
				}
			} else {
				_DBG_WARNING(("<cep 0x%p> Unable to resend RTU <lcid 0x%x rcid 0x%x>\n", _DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RemoteCommID));
			}
			break;
		case CMS_LAP_RCVD:
		case CMS_MRA_LAP_SENT:
		case CMS_DREQ_SENT:
		case CMS_DREQ_RCVD:
			goto discard;
			break;
		default: // timewait, SIM_DREQ_RCVD, SIDR, idle, listen
			// invalid LocalCommID
			goto nomatch;
			break;
	}

	_DBG_INFO(("<mad 0x%p> REP processed successfully <lcid 0x%x rcid 0x%x>!\n",
			_DBG_PTR(pMad), pMad->payload.REP.LocalCommID,
			pMad->payload.REP.RemoteCommID));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

nomatch:
	// REP did not match an active connection
	_DBG_WARNING(("<mad 0x%p> REP Rejected - no match found <lcid 0x%x rcid 0x%x>!\n", 
					_DBG_PTR(pMad), pMad->payload.REP.LocalCommID,
					pMad->payload.REP.RemoteCommID));

	SendREJ(RC_INVALID_COMMID, 
			CM_REJECT_REPLY, 
			pMad->common.TransactionID,
			0, 
			pMad->payload.REP.LocalCommID, // The remote end's local comm id
			pRemoteDgrm);

	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

fail:
	_DBG_WARNING(("<mad 0x%p> REP dropped - unable to process it! <lcid 0x%x rcid 0x%x %s>\n",
				_DBG_PTR(pMad), pMad->payload.REP.LocalCommID,
				pMad->payload.REP.RemoteCommID, _DBG_PTR(FSTATUS_MSG(Status))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

discard:
	_DBG_WARNING(("<mad 0x%p> REP discarded! <lcid 0x%x rcid 0x%x %s>\n",
				_DBG_PTR(pMad), pMad->payload.REP.LocalCommID,
				pMad->payload.REP.RemoteCommID, _DBG_PTR(CmGetStateString(pCEP->State))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

stale:
	_DBG_WARNING(("<mad 0x%p> REP discarded - stale! <lcid 0x%x rcid 0x%x %s>\n",
				_DBG_PTR(pMad), pMad->payload.REP.LocalCommID,
				pMad->payload.REP.RemoteCommID, _DBG_PTR(CmGetStateString(pCEP->State))));
	SendREJ(RC_STALE_CONN, CM_REJECT_REPLY, pMad->common.TransactionID,
				pCEP->LocalCommID, pMad->payload.REP.LocalCommID, pRemoteDgrm);
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// ProcessRtu()
// process an inbound RTU
//
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
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL. With ListLock held
//

FSTATUS
ProcessRtu(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pCEP=NULL;
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessRtu);

	_DBG_INFO(("<mad 0x%p> RTU received <lcid 0x%x rcid 0x%x>\n",					
					_DBG_PTR(pMad),
					pMad->payload.RTU.LocalCommID,
					pMad->payload.RTU.RemoteCommID));

	// Determine who received this CM MAD
	pCEP = FindLocalCommID(pMad->payload.RTU.RemoteCommID);
	if (! pCEP)
		goto nomatch;

	// TODO: Check the mad's addr with our endpoint's addr in addition to the
	// Local comm ID, also validate RemoteCommID
	// these should generate assorted REJ conditions if it doesn't match
	switch (pCEP->State)
	{
		// Connecting
		case CMS_REP_SENT:
		case CMS_MRA_REP_RCVD:
			// Process the rtu
			Status = ProcessRtuPassive(pCEP, pMad, pRemoteDgrm);
			if (Status == FSUCCESS)
			{
				_DBG_INFO(("<mad 0x%p> RTU processed successfully <lcid 0x%x rcid 0x%x>!\n",
					_DBG_PTR(pMad), pMad->payload.RTU.LocalCommID,
					pMad->payload.RTU.RemoteCommID));
			} else {
				_DBG_WARNING(("<mad 0x%p> RTU dropped - unable to process it! <lcid 0x%x rcid 0x%x %s>\n",
					_DBG_PTR(pMad), pMad->payload.RTU.LocalCommID,
					pMad->payload.RTU.RemoteCommID, _DBG_PTR(FSTATUS_MSG(Status))));
			}
			break;
		case CMS_REQ_SENT:
		case CMS_REP_WAIT:
		case CMS_REQ_RCVD:
		case CMS_MRA_REQ_SENT:
		case CMS_REP_RCVD:
		case CMS_MRA_REP_SENT:
			// discard RTU
			_DBG_WARNING(("<mad 0x%p> RTU Discarded - not expecting <lcid 0x%x rcid 0x%x>!\n", 
					_DBG_PTR(pMad), pMad->payload.RTU.LocalCommID,
					pMad->payload.RTU.RemoteCommID));
			break;
		// Connected
		case CMS_ESTABLISHED:
		case CMS_LAP_SENT:
		case CMS_MRA_LAP_RCVD:
		case CMS_LAP_RCVD:
		case CMS_MRA_LAP_SENT:
		case CMS_DREQ_SENT:
		case CMS_DREQ_RCVD:
			// discard RTU
			_DBG_WARNING(("<mad 0x%p> RTU Discarded - already connected <lcid 0x%x rcid 0x%x>!\n", 
					_DBG_PTR(pMad), pMad->payload.RTU.LocalCommID,
					pMad->payload.RTU.RemoteCommID));
			break;
		default: // timewait, SIM_DREQ_RCVD, SIDR, idle, listen
			// invalid LocalCommID
			goto nomatch;
			break;
	}

	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

nomatch:
	_DBG_WARNING(("<mad 0x%p> RTU dropped - no match found <lcid 0x%x rcid 0x%x>!\n",
					_DBG_PTR(pMad), pMad->payload.RTU.LocalCommID,
					pMad->payload.RTU.RemoteCommID));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}


//////////////////////////////////////////////////////////////////////////
// ProcessRtuPassive()
//
// This routine processes a RTU or a Async Connect Established event
// for the passive side of a connection
//
// pMad and pRemoteDgrm are NULL for the Async Connect Established event
//
// RETURNS:
//
//	FSUCCESS
//	FINVALID_STATE - QP not in CMS_REP_SENT/MRA_REP_RCVD
//			(most likely already connected)
//	FCM_INVALID_EVENT
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL.
//
FSTATUS
ProcessRtuPassive(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_MAD*					pMad,
	IN IBT_DGRM_ELEMENT*		pRemoteDgrm
	)
{
	FSTATUS Status = FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, AcceptCallback);

	if (pCEP->State != CMS_REP_SENT && pCEP->State != CMS_MRA_REP_RCVD)
	{
		Status = FINVALID_STATE;
		goto done;
	}

	Status = PrepareCepDgrm(pCEP);
	if (Status != FSUCCESS)
		goto fail;

	// Stop the rtu timer. Otherwise, we will retry REP or drop the pending request entirely.
	CmTimerStop(pCEP, REP_TIMER);
	// Save the rtu msg
#ifdef _CM_EVENT_ON_QP_
	// If we get a RTU event on the QP, we clear the mad so that
	// user knows there is no RTU payload available
	if ( pRemoteDgrm == NULL)
	{
		MemoryClear(GsiDgrmGetSendMad(pCEP->pDgrmElement), sizeof(CM_MAD));
	}
	else
#endif
	{
		CmDgrmCopyAddrAndMad(pCEP->pDgrmElement, pMad, pRemoteDgrm);
	}

	CepSetState(pCEP, CMS_ESTABLISHED);

	if (pCEP->bAsyncAccept)
	{
		SetNotification(pCEP, CME_RCVD_RTU);	// trigger callback
	} else {
		EventSet(pCEP, CME_RCVD_RTU, FALSE);	// wakeup CmAccept wait
	}

done:
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

fail:
	_DBG_WARNING(("<mad 0x%p> RTU dropped - unable to process it! <lcid 0x%x rcid 0x%x %s>\n",
				_DBG_PTR(pMad), pMad->payload.RTU.LocalCommID,
				pMad->payload.RTU.RemoteCommID, _DBG_PTR(FSTATUS_MSG(Status))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
} // ProcessRtuPassive()


//////////////////////////////////////////////////////////////////////////
// ProcessRejReq()
// process an inbound REJ of a REQ
//
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
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL. With ListLock held
//

FSTATUS
ProcessRejReq(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pCEP=NULL;
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessRejReq);

	// Rejecting a connect request or reply.
	_DBG_INFO(("<mad 0x%p> REJ REQ received <lcid 0x%x rcid 0x%x>\n",
					_DBG_PTR(pMad),
					pMad->payload.REJ.LocalCommID,
					pMad->payload.REJ.RemoteCommID));

	// Determine who received this CM MAD
	pCEP = FindLocalCommID(pMad->payload.REJ.RemoteCommID);
	if (! pCEP)
		goto nomatch;

	// TODO: Check the mad's addr with our endpoint's addr in addition to the
	// local comm ID, also validate RemoteCommID
	// these should generate assorted REJ conditions if it doesn't match
	switch (pCEP->State)
	{
		// Connecting
		case CMS_REQ_SENT:
		case CMS_REP_WAIT:
			// If we received a REJ with RC_INVALID_SID, we ignore it for peer
			// endpoint since we want to let the REQ timeout and resend.
			if (pCEP->bPeer && pMad->payload.REJ.Reason == RC_INVALID_SID)
				break;

			// Set the RemoteCommId based on the msg
			// pCEP->RemoteCommID = pMad->payload.REJ.LocalCommID;

			// Make sure we dont overwrite it if in used
			Status = PrepareCepDgrm(pCEP);
			if (Status != FSUCCESS)
				goto fail;

			// Cancel the timer on REQ
			CmTimerStop(pCEP, REQ_TIMER);
			// Save the reject msg. This will override the req msg
			CmDgrmCopyAddrAndMad(pCEP->pDgrmElement, pMad, pRemoteDgrm);
			if (pCEP->bPeer && pCEP->RemoteEndPoint.CaGUID == 0)
			{
				CM_MapRemoveEntry(&gCM->ListenMap, &pCEP->MapEntry, LISTEN_LIST, pCEP);
			}
			// Special case of rejection per IB spec
			if (pMad->payload.REJ.Reason == RC_STALE_CONN)
			{
				CepToTimewait(pCEP);
			} else {
				CepToIdle(pCEP);
			}

			// Update the event flag
			SetNotification(pCEP, CME_RCVD_REJ); //EventSet(pCEP, CME_RCVD_REJ);
			break;

		// the handling of a REJ while established (or in LAP states)
		// is tricky.  IBTA says to move to timewait
		// However sending the application an FCM_DISCONNECTED without a
		// FCM_DISCONNECT_REQUEST would confuse most applications.
		// Instead we simulate a FCM_DISCONNECT_REQUEST, and move to
		// SIM_DREQ_RCVD.  When the application issues the Disconnect we
		// will start the timewait
		case CMS_LAP_SENT:
		case CMS_MRA_LAP_RCVD:
			CmTimerStop(pCEP, LAP_TIMER);
			/* FALLSTHROUGH */
		case CMS_ESTABLISHED:
		case CMS_LAP_RCVD:
		case CMS_MRA_LAP_SENT:
			// If we received a REJ in a connected state, the other side did
			// not get the RTU. We need to treat this REJ as a DREQ.

			// Notify local user existing connection is disconnected
			// Fake out the REJ as a DREQ for the connection
			MemoryClear(pMad, sizeof(CM_MAD));
			pMad->payload.DREQ.LocalCommID = pCEP->RemoteCommID;
			pMad->payload.DREQ.RemoteCommID = pCEP->LocalCommID;
			pMad->payload.DREQ.u.s.RemoteQPNorEECN = pCEP->LocalEndPoint.QPN;
			// Save the simulated disconnect msg.
			CmDgrmCopyAddrAndMad(pCEP->pDgrmElement, pMad, pRemoteDgrm);

			// Move from CONNECTED to disconnecting
			CepToDisconnecting(pCEP, CMS_SIM_DREQ_RCVD);

			CmStartElapsedTime(pCEP);	// for turnaround computation
			SetNotification(pCEP, CME_SIM_RCVD_DREQ);
			break;
		default:
			goto discard;
			break;
	}
	_DBG_INFO(("<mad 0x%p> REJ REQ processed successfully <lcid 0x%x rcid 0x%x>!\n",
			_DBG_PTR(pMad), pMad->payload.REJ.LocalCommID, pMad->payload.REJ.RemoteCommID));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

nomatch:
	_DBG_WARNING(("<mad 0x%p> REJ REQ dropped - no match found <lcid 0x%x rcid 0x%x>!\n", 
					_DBG_PTR(pMad), pMad->payload.REJ.LocalCommID,
					pMad->payload.REJ.RemoteCommID));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
fail:
	_DBG_WARNING(("<mad 0x%p> REJ REQ dropped - unable to process it! <lcid 0x%x rcid 0x%x %s>\n",
					_DBG_PTR(pMad), pMad->payload.REJ.LocalCommID,
					pMad->payload.REJ.RemoteCommID, _DBG_PTR(FSTATUS_MSG(Status))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
discard:
	_DBG_WARNING(("<mad 0x%p> REJ REQ discarded! <lcid 0x%x rcid 0x%x %s>\n",
				_DBG_PTR(pMad), pMad->payload.REJ.LocalCommID,
				pMad->payload.REJ.RemoteCommID, _DBG_PTR(CmGetStateString(pCEP->State))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// ProcessRejRep()
// process an inbound REJ of a REP
//
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
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL. With ListLock held
//

FSTATUS
ProcessRejRep(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pCEP=NULL;
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessRejRep);

	// Rejecting a connect request or reply.
	_DBG_INFO(("<mad 0x%p> REJ REP received <lcid 0x%x rcid 0x%x>\n",
					_DBG_PTR(pMad),
					pMad->payload.REJ.LocalCommID,
					pMad->payload.REJ.RemoteCommID));

	// Determine who received this CM MAD
	pCEP = FindLocalCommID(pMad->payload.REJ.RemoteCommID);
	if (! pCEP)
		goto nomatch;

	// TODO: Check the mad's addr with our endpoint's addr in addition to the
	// local comm ID, also validate RemoteCommID
	// these should generate assorted REJ conditions if it doesn't match
	switch (pCEP->State)
	{
		// TBD - if in CMS_REQ_RCVD or CMD_MRA_REQ_SENT we have gotten a REJ
		// before we processed the REQ (timeout by client or cancel by client)
		// in which case we should move the CEP to idle (timewait should not
		// apply since we didn't get far enough to provide stale QPN)
		// user calls to iba_cm_accept or iba_cm_process_request should
		// then fail with some error (FCM_CONNECT_REJECT or FINVALID_STATE)
		// and cleanup the CEP.  While in this state, attempts to send REJ
		// or REP should fail or be ignored.
		case CMS_REP_SENT:
		case CMS_MRA_REP_RCVD:
			Status = PrepareCepDgrm(pCEP);
			if (Status != FSUCCESS)
				goto fail;
			// Stop the rtu timer. Otherwise, we will retry REP or drop the pending request entirely.
			CmTimerStop(pCEP, REP_TIMER);
			// Save the reject msg
			CmDgrmCopyAddrAndMad(pCEP->pDgrmElement, pMad, pRemoteDgrm);
			// Special case of rejection per IB spec
			if (pMad->payload.REJ.Reason == RC_STALE_CONN)
			{
				CepToTimewait(pCEP);
			} else {
				CepToIdle(pCEP);
			}
			if (pCEP->bAsyncAccept)
			{
				SetNotification(pCEP, CME_RCVD_REJ);	// callback
			} else {
				EventSet(pCEP, CME_RCVD_REJ, FALSE);	// wakeup CmAccept
			}
			break;
		default:
			goto discard;
			break;
	}

	_DBG_INFO(("<mad 0x%p> REJ REP processed successfully <lcid 0x%x rcid 0x%x>!\n",
			_DBG_PTR(pMad), pMad->payload.REJ.LocalCommID, pMad->payload.REJ.RemoteCommID));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

nomatch:
	_DBG_WARNING(("<mad 0x%p> REJ REP dropped - no match found <lcid 0x%x rcid 0x%x>!\n", 
					_DBG_PTR(pMad), pMad->payload.REJ.LocalCommID,
					pMad->payload.REJ.RemoteCommID));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
discard:
	_DBG_WARNING(("<mad 0x%p> REJ REQ discarded! <lcid 0x%x rcid 0x%x %s>\n",
				_DBG_PTR(pMad), pMad->payload.REJ.LocalCommID,
				pMad->payload.REJ.RemoteCommID, _DBG_PTR(CmGetStateString(pCEP->State))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
fail:
	_DBG_WARNING(("<mad 0x%p> REJ REP dropped - unable to process it! <lcid 0x%x rcid 0x%x %s>\n",
					_DBG_PTR(pMad), pMad->payload.REJ.LocalCommID,
					pMad->payload.REJ.RemoteCommID, _DBG_PTR(FSTATUS_MSG(Status))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// ProcessMraReq()
// process an inbound MRA of a REQ
//
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
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL. With ListLock held
//

FSTATUS
ProcessMraReq(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pCEP=NULL;
	FSTATUS Status=FSUCCESS;
	uint32 mra_timeout_ms=0;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessMraReq);

	// MRAed a connect request.
	_DBG_INFO(("<mad 0x%p> MRA REQ received <lcid 0x%x rcid 0x%x>\n",					
					_DBG_PTR(pMad),
					pMad->payload.MRA.LocalCommID,
					pMad->payload.MRA.RemoteCommID));

	// Determine who received this CM MAD
	pCEP = FindLocalCommID(pMad->payload.MRA.RemoteCommID);
	if (! pCEP)
		goto nomatch;
	// TODO: Check the mad's addr with our endpoint's addr in addition to the
	// local comm ID, also validate RemoteCommID
	// these should generate assorted REJ conditions if it doesn't match
	switch (pCEP->State)
	{
		// Connecting
		case CMS_REQ_SENT:
			// Received the mra to the connect request

			// Cancel the timer on REQ
			CmTimerStop(pCEP, REQ_TIMER);
			// Set the RemoteCommId based on the msg
			pCEP->RemoteCommID = pMad->payload.MRA.LocalCommID;
			gCM->turnaround_time_us = UpdateTurnaroundTime(gCM->turnaround_time_us,TimeoutMultToTimeInUsec(pMad->payload.MRA.ServiceTimeout));
			// restart the timer, do NOT reset RetryCount
			// allow for PktLifeTime in case MRA arrived faster than response
			mra_timeout_ms = TimeoutMultToTimeInMs(pMad->payload.MRA.ServiceTimeout)
							+ TimeoutMultToTimeInMs(pCEP->PktLifeTime);
			CmTimerStart(pCEP, mra_timeout_ms, MRA_TIMER);
			_DBG_INFO(("<mad 0x%p> MRA REQ processed successfully <lcid 0x%x rcid 0x%x>!\n",
				_DBG_PTR(pMad), pMad->payload.MRA.LocalCommID,
				pMad->payload.MRA.RemoteCommID));
			CepSetState(pCEP, CMS_REP_WAIT);
			break;

		case CMS_REP_WAIT:
		case CMS_REQ_RCVD:
		case CMS_MRA_REQ_SENT:
		case CMS_REP_SENT:
		case CMS_MRA_REP_RCVD:
		case CMS_REP_RCVD:
		case CMS_MRA_REP_SENT:
		default: // not connecting
			goto discard;
			break;
	}

	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

nomatch:
	_DBG_WARNING(("<mad 0x%p> MRA REQ dropped - no match found <lcid 0x%x rcid 0x%x>!\n", 
					_DBG_PTR(pMad), pMad->payload.MRA.LocalCommID,
					pMad->payload.MRA.RemoteCommID));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

discard:
	_DBG_WARNING(("<mad 0x%p> MRA REQ discarded! <lcid 0x%x rcid 0x%x %s>\n",
				_DBG_PTR(pMad), pMad->payload.MRA.LocalCommID,
				pMad->payload.MRA.RemoteCommID, _DBG_PTR(CmGetStateString(pCEP->State))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}


//////////////////////////////////////////////////////////////////////////
// ProcessMraRep()
// process an inbound MRA of a REP
//
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
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL. With ListLock held
//

FSTATUS
ProcessMraRep(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pCEP=NULL;
	FSTATUS Status=FSUCCESS;
	uint32 mra_timeout_ms=0;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessMraRep);

	// MRAed a connect reply.
	_DBG_INFO(("<mad 0x%p> MRA REP received <lcid 0x%x rcid 0x%x>\n",					
					_DBG_PTR(pMad),
					pMad->payload.MRA.LocalCommID,
					pMad->payload.MRA.RemoteCommID));

	// Determine who received this CM MAD
	pCEP = FindLocalCommID(pMad->payload.MRA.RemoteCommID);
	if (! pCEP)
		goto nomatch;
	// TODO: Check the mad's addr with our endpoint's addr in addition to the
	// local comm ID, also validate RemoteCommID
	// these should generate assorted REJ conditions if it doesn't match
	switch (pCEP->State)
	{
		case CMS_REP_SENT:
			// Stop the rtu timer. Otherwise, we will retry REP or drop the pending request entirely.
			CmTimerStop(pCEP, REP_TIMER);

			gCM->turnaround_time_us = UpdateTurnaroundTime(gCM->turnaround_time_us,TimeoutMultToTimeInUsec(pMad->payload.MRA.ServiceTimeout));
			// restart the timer, do NOT reset RetryCount
			// allow for PktLifeTime in case MRA arrived faster than response
			mra_timeout_ms = TimeoutMultToTimeInMs(pMad->payload.MRA.ServiceTimeout)
							+ TimeoutMultToTimeInMs(pCEP->PktLifeTime);
			CmTimerStart(pCEP, mra_timeout_ms, MRA_TIMER);
			CepSetState(pCEP, CMS_MRA_REP_RCVD);
			_DBG_INFO(("<mad 0x%p> MRA REP processed successfully <lcid 0x%x rcid 0x%x>!\n",
				_DBG_PTR(pMad), pMad->payload.MRA.LocalCommID,
				pMad->payload.MRA.RemoteCommID));
			break;
		default:
			goto discard;
			break;
	}

	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

nomatch:
	_DBG_WARNING(("<mad 0x%p> MRA REP dropped - no match found <lcid 0x%x rcid 0x%x>!\n", 
			_DBG_PTR(pMad), pMad->payload.MRA.LocalCommID,
			pMad->payload.MRA.RemoteCommID));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
discard:
	_DBG_WARNING(("<mad 0x%p> MRA REP discarded! <lcid 0x%x rcid 0x%x %s>\n",
				_DBG_PTR(pMad), pMad->payload.MRA.LocalCommID,
				pMad->payload.MRA.RemoteCommID, _DBG_PTR(CmGetStateString(pCEP->State))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// ProcessMraLap()
// process an inbound MRA of a LAP
//
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
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL. With ListLock held
//

FSTATUS
ProcessMraLap(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pCEP=NULL;
	FSTATUS Status=FSUCCESS;
	uint32 mra_timeout_ms=0;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessMraLap);

	// MRAed a LAP.
	_DBG_INFO(("<mad 0x%p> MRA LAP received <lcid 0x%x rcid 0x%x>\n",					
					_DBG_PTR(pMad),
					pMad->payload.MRA.LocalCommID,
					pMad->payload.MRA.RemoteCommID));

	// Determine who received this CM MAD
	pCEP = FindLocalCommID(pMad->payload.MRA.RemoteCommID);
	if (! pCEP)
		goto nomatch;
	switch (pCEP->State)
	{
		case CMS_LAP_SENT:
			// Received the mra to the LAP

			// Cancel the timer on LAP
			CmTimerStop(pCEP, LAP_TIMER);

			gCM->turnaround_time_us = UpdateTurnaroundTime(gCM->turnaround_time_us,TimeoutMultToTimeInUsec(pMad->payload.MRA.ServiceTimeout));
			// restart the timer, do NOT reset RetryCount
			// allow for PktLifeTime in case MRA arrived faster than response
			mra_timeout_ms = TimeoutMultToTimeInMs(pMad->payload.MRA.ServiceTimeout)
							+ TimeoutMultToTimeInMs(pCEP->PktLifeTime);
			CmTimerStart(pCEP, mra_timeout_ms, MRA_TIMER);
			_DBG_INFO(("<mad 0x%p> MRA LAP processed successfully <lcid 0x%x rcid 0x%x>!\n",
				_DBG_PTR(pMad), pMad->payload.MRA.LocalCommID,
				pMad->payload.MRA.RemoteCommID));
			CepSetState(pCEP, CMS_MRA_LAP_RCVD);
			break;

		case CMS_MRA_LAP_RCVD:
		case CMS_LAP_RCVD:
		case CMS_MRA_LAP_SENT:
		default:
			goto discard;
			break;
	}
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

nomatch:
	_DBG_WARNING(("<mad 0x%p> MRA LAP dropped - no match found <lcid 0x%x rcid 0x%x>!\n", 
				_DBG_PTR(pMad), pMad->payload.MRA.LocalCommID,
				pMad->payload.MRA.RemoteCommID));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

discard:
	_DBG_WARNING(("<mad 0x%p> MRA LAP discarded! <lcid 0x%x rcid 0x%x %s>\n",
				_DBG_PTR(pMad), pMad->payload.MRA.LocalCommID,
				pMad->payload.MRA.RemoteCommID, _DBG_PTR(CmGetStateString(pCEP->State))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// ProcessLap()
// process an inbound LAP
//
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
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL. With ListLock held
//

FSTATUS
ProcessLap(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pCEP=NULL;
	FSTATUS Status=FSUCCESS;
	uint32	worst_service_time_mult=0;
	uint64  new_turnaround;
	IB_PORT_ATTRIBUTES *pPortAttr = NULL;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessLap);

	_DBG_INFO(("<mad 0x%p> LAP received <lcid 0x%x rcid 0x%x>\n",					
					_DBG_PTR(pMad),
					pMad->payload.LAP.LocalCommID,
					pMad->payload.LAP.RemoteCommID));

	// Determine who received this CM MAD
	pCEP = FindLocalCommID(pMad->payload.LAP.RemoteCommID);
	if (! pCEP)
		goto nomatch;

	// TODO: Check the mad's addr with our endpoint's addr in addition to the
	// Local comm ID, also validate RemoteCommID
	// these should generate assorted REJ conditions if it doesn't match
	switch (pCEP->State)
	{
		case CMS_REP_SENT:
		case CMS_MRA_REP_RCVD:
			// Must have lost the RTU, treat the LAP as an RTU and move forward
			Status = ProcessRtuPassive(pCEP, NULL, NULL);
			if (Status == FSUCCESS)
			{
				_DBG_INFO(("LAP recognized as RTU+LAP <cep 0x%p>!\n", _DBG_PTR(pCEP)));
			} else if (Status != FINVALID_STATE) {
				goto fail;
			}
			/*FALLSTHROUGH*/
		case CMS_ESTABLISHED:
			if (pCEP->Mode != CM_MODE_PASSIVE)
				goto discard;	// TBD could send APR rejection

			if (! pCEP->bFailoverSupported)
			{
				SendAPR(APS_UNSUPPORTED_REQ, pMad->common.TransactionID,
						pCEP->LocalCommID, pCEP->RemoteCommID, pRemoteDgrm);
				goto done;
			}

			if (0 == CompareRcvdLAPToPath(&pMad->payload.LAP, &pCEP->PrimaryPath))
			{
				SendAPR(APS_DUPLICATE_PATH, pMad->common.TransactionID,
						pCEP->LocalCommID, pCEP->RemoteCommID, pRemoteDgrm);
				goto done;
			}
			if (0 == CompareRcvdLAPToPath(&pMad->payload.LAP, &pCEP->AlternatePath))
			{
				// duplicate LAP, respond again
				SendAPR(APS_PATH_LOADED, pMad->common.TransactionID,
						pCEP->LocalCommID, pCEP->RemoteCommID, pRemoteDgrm);
				goto done;
			}
#if ! defined(CM_APM_RELOAD)
			if (pCEP->AlternatePath.LocalLID)
			{
				// reject, we can't reload a new alternate til we migrate
				SendAPR(APS_REJECTED, pMad->common.TransactionID,
						pCEP->LocalCommID, pCEP->RemoteCommID, pRemoteDgrm);
				goto done;
			}
#endif
			Status = iba_find_ca_gid(&pMad->payload.LAP.AlternateRemoteGID,
					pCEP->LocalEndPoint.CaGUID,
					&pCEP->TempAlternateLocalPortGuid,
					&pCEP->TempAlternateLocalGidIndex, &pPortAttr, NULL);
			if (Status != FSUCCESS)
			{
				SendAPR(APS_REJECT_DGID, pMad->common.TransactionID,
						pCEP->LocalCommID, pCEP->RemoteCommID, pRemoteDgrm);
				goto done;
			}
			if (! ValidatePKey(pPortAttr, pCEP->PartitionKey, &pCEP->TempAlternatePkeyIndex))
			{
				SendAPR(APS_REJECT_DGID, pMad->common.TransactionID,
						pCEP->LocalCommID, pCEP->RemoteCommID, pRemoteDgrm);
				goto done;
			}
			pCEP->TempAlternateLocalPathBits = LidToPathBits(pMad->payload.LAP.AlternateRemoteLID, pPortAttr->Address.LMC);
			MemoryDeallocate(pPortAttr);

			Status = PrepareCepDgrm(pCEP);
			if (Status != FSUCCESS)
				goto fail;

			CmDgrmCopyAddrAndMad(pCEP->pDgrmElement, pMad, pRemoteDgrm);

			CepSetState(pCEP, CMS_LAP_RCVD);

			// compare RemoteCMTimeout to our typical turnaround
			// if necessary, output an MRA to be safe
			worst_service_time_mult = CepWorstServiceTimeMult(pCEP);
			if (pMad->payload.LAP.u1.s.RemoteCMTimeout < worst_service_time_mult) {
				SendMRA(CM_MRA_LAP, worst_service_time_mult,
							pCEP->TransactionID, pCEP->LocalCommID,
							pCEP->RemoteCommID, pRemoteDgrm);
				CepSetState(pCEP, CMS_MRA_LAP_SENT);
			}
			SetNotification(pCEP, CME_RCVD_LAP);	// trigger callback
			break;
		case CMS_LAP_RCVD:
		case CMS_MRA_LAP_SENT:
			ASSERT(pCEP->Mode == CM_MODE_PASSIVE);
			if (0 == CompareRcvdLAPToPath(&pMad->payload.LAP, &pCEP->PrimaryPath))
			{
				SendAPR(APS_DUPLICATE_PATH, pMad->common.TransactionID,
						pCEP->LocalCommID, pCEP->RemoteCommID, pRemoteDgrm);
				goto done;
			}
			if (0 == CompareLAP(&pMad->payload.LAP,
						&((CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement))->payload.LAP))
			{
				// duplicate LAP
				// Update the turnaround time
				new_turnaround = UpdateTurnaroundTime(gCM->turnaround_time_us,  2*CmGetElapsedTime(pCEP));
				if (new_turnaround > gCM->turnaround_time_us)
				{
					gCM->turnaround_time_us = new_turnaround;
				}
				worst_service_time_mult = CepWorstServiceTimeMult(pCEP);
				if (pCEP->State == CMS_MRA_LAP_SENT)
				{
					SendMRA(CM_MRA_LAP, worst_service_time_mult,
								pCEP->TransactionID, pCEP->LocalCommID,
								pCEP->RemoteCommID, pRemoteDgrm);
					goto done;
				} else if (pMad->payload.LAP.u1.s.RemoteCMTimeout < worst_service_time_mult) {
					SendMRA(CM_MRA_LAP, worst_service_time_mult,
					pCEP->TransactionID, pCEP->LocalCommID,
						pCEP->RemoteCommID, pRemoteDgrm);
					CepSetState(pCEP, CMS_MRA_LAP_SENT);
  					goto done;
				} else {
					goto discard;
				}
			}

			// TBD or disconnect
			// if we are really slow, other end could timeout LAP
			// and this could be a new LAP, in which case this APR/reject
			// will happen, then we will send an APR and remote end will
			// discard it, leaving local end with an alternate path
			// and remote end with no alternate path
			SendAPR(APS_REJECTED, pMad->common.TransactionID,
					pCEP->LocalCommID, pCEP->RemoteCommID, pRemoteDgrm);
			break;
		default:
			goto discard;
			break;
	}

done:
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

fail:
	_DBG_WARNING(("<mad 0x%p> LAP dropped - unable to process it! <lcid 0x%x rcid 0x%x %s>\n",
					_DBG_PTR(pMad), pMad->payload.LAP.LocalCommID,
					pMad->payload.LAP.RemoteCommID, _DBG_PTR(FSTATUS_MSG(Status))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

discard:
	_DBG_WARNING(("<mad 0x%p> LAP Discarded <lcid 0x%x rcid 0x%x>!\n", 
			_DBG_PTR(pMad), pMad->payload.LAP.LocalCommID,
			pMad->payload.LAP.RemoteCommID));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

nomatch:
	_DBG_WARNING(("<mad 0x%p> LAP dropped - no match found <lcid 0x%x rcid 0x%x>!\n",
					_DBG_PTR(pMad), pMad->payload.LAP.LocalCommID,
					pMad->payload.LAP.RemoteCommID));
	SendAPR(APS_INVALID_COMMID, pMad->common.TransactionID,
					pMad->payload.LAP.RemoteCommID,
					pMad->payload.LAP.LocalCommID, pRemoteDgrm);
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}


//////////////////////////////////////////////////////////////////////////
// ProcessApr()
// process an inbound APR
//
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
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL. With ListLock held
//

FSTATUS
ProcessApr(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pCEP=NULL;
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessApr);

	_DBG_INFO(("<mad 0x%p> APR received <lcid 0x%x rcid 0x%x APstatus %u>\n",					
					_DBG_PTR(pMad),
					pMad->payload.APR.LocalCommID,
					pMad->payload.APR.RemoteCommID,
					pMad->payload.APR.APStatus));

	// Determine who received this CM MAD
	pCEP = FindLocalCommID(pMad->payload.LAP.RemoteCommID);
	if (! pCEP)
		goto nomatch;

	// TODO: Check the mad's addr with our endpoint's addr in addition to the
	// Local comm ID, also validate RemoteCommID
	// these should generate assorted REJ conditions if it doesn't match
	switch (pCEP->State)
	{
		case CMS_LAP_SENT:
		case CMS_MRA_LAP_RCVD:
			ASSERT(pCEP->Mode == CM_MODE_ACTIVE);

			Status = PrepareCepDgrm(pCEP);
			if (Status != FSUCCESS)
				goto fail;

			if (pMad->payload.APR.APStatus == APS_PATH_LOADED)
			{
				// Dgrm was byteswapped for send
				ConvertMadToHostByteOrder(pMad);

				ASSERT(((CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement))->common.AttributeID == MCLASS_ATTRIB_ID_LAP);
				CopySentLAPToAltPath(
					&((CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement))->payload.LAP,
					&pCEP->AlternatePath);
				pCEP->AlternatePath.LocalPortGuid = pCEP->TempAlternateLocalPortGuid;
				pCEP->AlternatePath.LocalGidIndex = pCEP->TempAlternateLocalGidIndex;
				pCEP->AlternatePath.LocalPathBits = pCEP->TempAlternateLocalPathBits;
				pCEP->AlternatePath.PkeyIndex = pCEP->TempAlternatePkeyIndex;
				pCEP->AlternatePath.LocalAckTimeout = pCEP->TempAlternateLocalAckTimeout;
			}

			//Save the APR
			CmDgrmCopyAddrAndMad(pCEP->pDgrmElement, pMad, pRemoteDgrm);

			CmTimerStop(pCEP, LAP_TIMER);
			CepSetState(pCEP, CMS_ESTABLISHED);
			SetNotification(pCEP, CME_RCVD_APR);
			_DBG_INFO(("<mad 0x%p> APR processed successfully <lcid 0x%x rcid 0x%x>!\n",
					_DBG_PTR(pMad), pMad->payload.APR.LocalCommID,
					pMad->payload.APR.RemoteCommID));
			break;
		default:
			goto discard;
			break;
	}

	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

discard:
	_DBG_WARNING(("<mad 0x%p> APR Discarded <lcid 0x%x rcid 0x%x>!\n", 
			_DBG_PTR(pMad), pMad->payload.APR.LocalCommID,
			pMad->payload.APR.RemoteCommID));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

fail:
	_DBG_WARNING(("<mad 0x%p> APR dropped - unable to process it! <lcid 0x%x rcid 0x%x %s>\n",
					_DBG_PTR(pMad), pMad->payload.APR.LocalCommID,
					pMad->payload.APR.RemoteCommID, _DBG_PTR(FSTATUS_MSG(Status))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

nomatch:
	// TBD - send APR with APStatus = 1
	_DBG_WARNING(("<mad 0x%p> APR dropped - no match found <lcid 0x%x rcid 0x%x>!\n",
					_DBG_PTR(pMad), pMad->payload.APR.LocalCommID,
					pMad->payload.APR.RemoteCommID));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}


//////////////////////////////////////////////////////////////////////////
// ProcessDreq()
// process an inbound DREQ
//
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
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL. With ListLock held
//

FSTATUS
ProcessDreq(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pCEP=NULL;
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessDreq);

	//
	// Disconnect request - apply to ESTABLISHED list
	//
	_DBG_INFO(("<mad 0x%p> DREQ received <lcid 0x%x rcid 0x%x>\n",					
					_DBG_PTR(pMad),
					pMad->payload.DREQ.LocalCommID,
					pMad->payload.DREQ.RemoteCommID));

	// Determine who received this CM MAD
	pCEP = FindLocalCommID(pMad->payload.DREQ.RemoteCommID);
	if (! pCEP)
		goto nomatch;

	// TODO: Check the mad's addr with our endpoint's addr in addition to the
	// local comm ID, also validate RemoteCommID
	// these should generate assorted REJ/error conditions if it doesn't match
	switch (pCEP->State)
	{
		// Connected
		case CMS_ESTABLISHED:
		case CMS_LAP_SENT:
		case CMS_MRA_LAP_RCVD:
		case CMS_LAP_RCVD:
		case CMS_MRA_LAP_SENT:
		// TimeWait
		case CMS_DREQ_SENT:
		case CMS_DREQ_RCVD:
		case CMS_SIM_DREQ_RCVD: // this is a little odd, since we got a REJ
								// then a DREQ for connection, but it is
								// what IBTA CM state machine says to do
		case CMS_TIMEWAIT:
			// Process the DREQ
			Status = ProcessDreqCep(pCEP, pMad, pRemoteDgrm);
			if (Status == FSUCCESS)
			{
				_DBG_INFO(("<mad 0x%p> DREQ processed successfully <lcid 0x%x rcid 0x%x>!\n", 
						_DBG_PTR(pMad), pMad->payload.DREQ.LocalCommID,
						pMad->payload.DREQ.RemoteCommID));
			} else {
				_DBG_WARNING(("<mad 0x%p> DREQ dropped - unable to processed it! <lcid 0x%x rcid 0x%x %s>\n",
						_DBG_PTR(pMad), pMad->payload.DREQ.LocalCommID,
						pMad->payload.DREQ.RemoteCommID, _DBG_PTR(FSTATUS_MSG(Status))));
			}
			break;
		default:
			goto nomatch;
			break;
	}
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

nomatch:
	_DBG_WARNING(("<mad 0x%p> DREQ dropped - no match found <lcid 0x%x rcid 0x%x>!\n",
				_DBG_PTR(pMad), pMad->payload.DREQ.LocalCommID,
				pMad->payload.DREQ.RemoteCommID));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// ProcessDreqCep()
//
// Process a valid DREQ received or simulated against a specific CEP.
// The ListLock must be obtained before calling this routine
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
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
ProcessDreqCep(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_MAD*					pMad,
	IN IBT_DGRM_ELEMENT*		pRemoteDgrm
	)
{
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessDreqCep);

	switch (pCEP->State)
	{
		case CMS_LAP_SENT:
		case CMS_MRA_LAP_RCVD:
			CmTimerStop(pCEP, LAP_TIMER);
			/* FALLSTHROUGH */
		case CMS_ESTABLISHED:
		case CMS_LAP_RCVD:
		case CMS_MRA_LAP_SENT:
			// Save the disconnect msg.
			CmDgrmCopyAddrAndMad(pCEP->pDgrmElement, pMad, pRemoteDgrm);

			// Move from CONNECTED to Disconnecting
			CepToDisconnecting(pCEP, CMS_DREQ_RCVD);

			CmStartElapsedTime(pCEP);	// for turnaround computation
			// RTU followed immediately by a DREQ may queue 2 events
			SetNotification(pCEP, CME_RCVD_DREQ);
			break;
		case CMS_DREQ_RCVD:
			// Duplicate DREQ received.
			_DBG_INFO(("<cep 0x%p> Duplicate DREQ received in CMS_DREQ_RCVD <lcid 0x%x rcid 0x%x slid 0x%x dlid 0x%x>.\n", 
					 _DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RemoteCommID,
					 pCEP->PrimaryPath.LocalLID, pCEP->PrimaryPath.RemoteLID));
			break;
		case CMS_DREQ_SENT:
			_DBG_INFO(("<cep 0x%p> DREQ received in CMS_DREQ_SENT <lcid 0x%x rcid 0x%x slid 0x%x dlid 0x%x>.\n", 
					 _DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RemoteCommID,
					 pCEP->PrimaryPath.LocalLID, pCEP->PrimaryPath.RemoteLID));

			Status = PrepareCepDgrm(pCEP);
			if (Status != FSUCCESS)
				goto fail;

			CmTimerStop(pCEP, DREQ_TIMER);
			// Save the disconnect request
			CmDgrmCopyAddrAndMad(pCEP->pDgrmElement, pMad, pRemoteDgrm);
			CepSetState(pCEP, CMS_DREQ_RCVD);
			// Note that we may have reached the DREQ sent state via a
			// CmDestroyCEP call from the proxy on behalf of a crashed app.
			// In this case, we must continue the disconnect process or the CEP
			// will never transition out of its current state.
			if( !SetNotification(pCEP, CME_RCVD_DREQ) )
			{
				CM_DREQUEST_INFO		DisconnectReq;
				MemoryClear(&DisconnectReq, sizeof(CM_DREQUEST_INFO));
				DisconnectA(pCEP, &DisconnectReq, (CM_DREPLY_INFO*)&DisconnectReq);
			}
			break;
		case CMS_SIM_DREQ_RCVD:
		case CMS_TIMEWAIT:
			{
			// Resend the DREP

			// If dgrm is still in use (i.e. send pending), skip the resend
			// since it is likely the timeout is set too low
			if (CmDgrmIsInUse(pCEP->pDgrmElement))
				goto done;

			if (GsiDgrmGetSendMad(pCEP->pDgrmElement)->common.AttributeID != MCLASS_ATTRIB_ID_DREP )
			{
				// Format the DREP msg
				FormatDREP((CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement),
							NULL, 0,
							pCEP->TransactionID,
							pCEP->LocalCommID,
							pCEP->RemoteCommID);
			}

			_DBG_INFO(("<cep 0x%p> DREP resend in %s <lcid 0x%x slid 0x%x dlid 0x%x>.\n", 
					_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State)),
					pCEP->LocalCommID,
					pCEP->PrimaryPath.LocalLID, pCEP->PrimaryPath.RemoteLID));
			Status = CmDgrmSend(pCEP->pDgrmElement);
			if (FSUCCESS != Status)
			{
				// fall through and let timer retry the send later
				_DBG_WARN(("<cep 0x%p> DgrmSend() failed for DREP!!! <%s>\n",
							_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
				Status = FSUCCESS;
			} else {
				AtomicIncrementVoid(&gCM->Sent.Drep);
			}
			break;
			}
		default:
			break;
	}

done:
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

fail:
	_DBG_WARNING(("<mad 0x%p> DREQ dropped - unable to process it! <lcid 0x%x rcid 0x%x %s>\n",
				_DBG_PTR(pMad), pMad->payload.DREQ.LocalCommID,
				pMad->payload.DREQ.RemoteCommID, _DBG_PTR(FSTATUS_MSG(Status))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

} // ProcessDreqCep()


//////////////////////////////////////////////////////////////////////////
// ProcessDrep()
// process an inbound DREP
//
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
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL. With ListLock held
//

FSTATUS
ProcessDrep(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pCEP=NULL;
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessDrep);

	//
	//  Disconnect reply received - apply to TIMEWAIT list
	//
	_DBG_INFO(("<mad 0x%p> DREP received <lcid 0x%x rcid 0x%x>\n",					
					_DBG_PTR(pMad),
					pMad->payload.DREP.LocalCommID,
					pMad->payload.DREP.RemoteCommID));

	// Determine who received this CM MAD
	pCEP = FindLocalCommID(pMad->payload.DREP.RemoteCommID);
	if (! pCEP)
		goto nomatch;

	// TODO: Check the mad's addr with our endpoint's addr in addition to the
	// local comm ID, also validate RemoteCommID
	// these should generate assorted REJ/error conditions if it doesn't match
	switch (pCEP->State)
	{
		case CMS_DREQ_SENT:
			Status = PrepareCepDgrm(pCEP);
			if (Status != FSUCCESS)
				goto fail;

			//Save the disconnect reply
			CmDgrmCopyAddrAndMad(pCEP->pDgrmElement, pMad, pRemoteDgrm);

			CmTimerStop(pCEP, DREQ_TIMER);
			CepToTimewait(pCEP);
			SetNotification(pCEP, CME_RCVD_DREP);
			_DBG_INFO(("<mad 0x%p> DREP processed successfully <lcid 0x%x rcid 0x%x>!\n",
					_DBG_PTR(pMad), pMad->payload.DREP.LocalCommID,
					pMad->payload.DREP.RemoteCommID));
			break;
		case CMS_DREQ_RCVD:
		case CMS_SIM_DREQ_RCVD:
		case CMS_TIMEWAIT:
		default:
			goto discard;
			break;
	}
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

nomatch:
	_DBG_WARNING(("<mad 0x%p> DREP dropped - no match found <lcid 0x%x rcid 0x%x>!\n",
				_DBG_PTR(pMad), pMad->payload.DREP.LocalCommID,
				pMad->payload.DREP.RemoteCommID));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

discard:
	_DBG_WARNING(("<mad 0x%p> DREP discarded! <lcid 0x%x rcid 0x%x %s>\n",
				_DBG_PTR(pMad), pMad->payload.DREP.LocalCommID,
				pMad->payload.DREP.RemoteCommID, _DBG_PTR(CmGetStateString(pCEP->State))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

fail:
	_DBG_WARNING(("<mad 0x%p> DREP dropped - unable to processed it! <lcid 0x%x rcid 0x%x %s>\n",
				_DBG_PTR(pMad), pMad->payload.DREP.LocalCommID,
				pMad->payload.DREP.RemoteCommID, _DBG_PTR(FSTATUS_MSG(Status))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// ProcessSidrReq()
//
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
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL. With ListLock held
//

FSTATUS
ProcessSidrReq(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pCEP=NULL;
	CM_MAP_ITEM* pMapEntry = NULL;
	boolean	bFound=FALSE;
	FSTATUS Status=FSUCCESS;
	CM_CEP_OBJECT* pNewCEP=NULL;
	DLIST_ENTRY* Anchor = NULL;
	DLIST_ENTRY* Index = NULL;
	boolean bDuplicate = FALSE;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessSidrReq);

	//
	// SIDR request received - apply to LISTEN list
	//
	_DBG_INFO(("<mad 0x%p> SIDR_REQ received <reqid 0x%x sid 0x%"PRIx64">\n",					
					_DBG_PTR(pMad),
					pMad->payload.SIDR_REQ.RequestID,
					pMad->payload.SIDR_REQ.ServiceID));

	// Look for the CEP object in the listen list
	pMapEntry = CM_MapGetCompare(&gCM->ListenMap, (uintn)&pMad->payload.SIDR_REQ,
							CepSidrReqAddrCompare);
	if (pMapEntry != CM_MapEnd(&gCM->ListenMap))
	{
		pCEP = PARENT_STRUCT(pMapEntry, CM_CEP_OBJECT, MapEntry);
		if (pCEP->State == CMS_REGISTERED)
		{
			bFound = TRUE;
		}
	}

	if (!bFound)
	{
		_DBG_WARNING(("<mad 0x%p> SIDR REQ dropped - no match found <reqid 0x%x sid 0x%"PRIx64"> !\n", 
					_DBG_PTR(pMad), pMad->payload.SIDR_REQ.RequestID, pMad->payload.SIDR_REQ.ServiceID));

		SendSIDR_RESP(pMad->payload.SIDR_REQ.ServiceID, 
						0, 0, 
						SRS_SID_NOT_SUPPORTED,
						NULL, 0, pMad->common.TransactionID,
						pMad->payload.SIDR_REQ.RequestID,
						pRemoteDgrm);
		goto done;
	}

	// Check for duplicate request in PENDING list
	DListIterate(Anchor, Index, &pCEP->PendingList)
	{
		CM_CEP_OBJECT* pCEPEntry;

		pCEPEntry = PARENT_STRUCT(Index, CM_CEP_OBJECT, PendingListEntry);

		// We have to check GID, LID and RequestId to determine if
		// this is a duplicate, most likely a retry by requestor
		if ( (pCEPEntry->PrimaryPath.RemoteLID == pRemoteDgrm->RemoteLID) &&
			(! (pRemoteDgrm->GlobalRoute) || (MemoryCompare(pCEPEntry->PrimaryPath.RemoteGID.Raw, pRemoteDgrm->GRHInfo.DestGID.Raw, sizeof(IB_GID)) == 0)) &&
			 (pCEPEntry->RemoteCommID == pMad->payload.SIDR_REQ.RequestID))
		{
			bDuplicate = TRUE;
			break;
		}
	}

	if (bDuplicate)
	{
		_DBG_WARNING(("<mad 0x%p> SIDR REQ dropped - duplicates pending request! <reqid 0x%x sid 0x%"PRIx64" %s>\n",
					_DBG_PTR(pMad), pMad->payload.SIDR_REQ.RequestID,
					pMad->payload.SIDR_REQ.ServiceID, _DBG_PTR(FSTATUS_MSG(Status))));
		//MsgOut("*** SIDR REQ Recv DUP Dropped ***, Remote Lid=0x%x\n", pRemoteDgrm->RemoteLID);
		Status = FDUPLICATE;
		goto done;
	}

	// If we hit this backlog cnt, start dropping the incoming requests
	// since this SIDR endpoint maybe dead or very slow
	if (pCEP->PendingCount >= (pCEP->ListenBackLog?pCEP->ListenBackLog:gCmParams.MaxBackLog))
	{
		_DBG_ERROR(("<cep 0x%p> Max pending count reached!!! <sid 0x%"PRIx64" lcid 0x%x rcid 0x%x max pend cnt %d>\n", 
					_DBG_PTR(pCEP), pMad->payload.SIDR_REQ.ServiceID,
					pCEP->LocalCommID, pMad->payload.SIDR_REQ.RequestID,
					pCEP->PendingCount));

		Status = FOVERRUN;
		goto dropreq;
	}

	// Does the user want to be notified
	if (! pCEP->bSidrRegisterNotify)
	{
		// User does not want to be notified. Send the SIDR_RESP.
		Status = SendSIDR_RESP(pCEP->SID, 
									pCEP->LocalEndPoint.QPN, 
									pCEP->QKey, 
									SRS_VALID_QPN,
									NULL, 0,
									pMad->common.TransactionID,
									pMad->payload.SIDR_REQ.RequestID,
									pRemoteDgrm);
	} else {
		// Create a new object to represent this SIDR_REQ
		pNewCEP = CreateCEP(CM_UD_TYPE, NULL, NULL);
		if (pNewCEP == NULL)
		{
			Status = FINSUFFICIENT_RESOURCES;
			goto dropreq;
		}

		// Queue the request onto the "registered" object
		CepAddToPendingList(pCEP, pNewCEP);

		CepSetState(pNewCEP, CMS_SIDR_REQ_RCVD);

		// Save the request msg to the new CEP obj.
		CmDgrmCopyAddrAndMad(pNewCEP->pDgrmElement, pMad, pRemoteDgrm);

		(void)iba_get_ca_for_port(pRemoteDgrm->PortGuid, &pNewCEP->LocalEndPoint.CaGUID);
		// Save the request id to reply to
		pNewCEP->RemoteCommID = pMad->payload.SIDR_REQ.RequestID;
		pNewCEP->TransactionID = pMad->common.TransactionID;

		pNewCEP->SID = pMad->payload.SIDR_REQ.ServiceID;
		pNewCEP->PrimaryPath.RemoteLID = pRemoteDgrm->RemoteLID;
		if (pRemoteDgrm->GlobalRoute) {
			MemoryCopy(pNewCEP->PrimaryPath.RemoteGID.Raw, pRemoteDgrm->GRHInfo.DestGID.Raw, sizeof(IB_GID));
		}

		// Signal the registered object for user callback
		SetNotification(pCEP, CME_RCVD_SIDR_REQ);
	}

	if (Status == FSUCCESS)
	{
		_DBG_INFO(("<mad 0x%p> SIDR REQ processed successfully <reqid 0x%x sid 0x%"PRIx64">!\n",
					_DBG_PTR(pMad), pMad->payload.SIDR_REQ.RequestID, pMad->payload.SIDR_REQ.ServiceID));
	} else {
		goto dropreq;
	}

done:
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

dropreq:
	_DBG_WARNING(("<mad 0x%p> SIDR REQ dropped - unable to process it! <reqid 0x%x sid 0x%"PRIx64" %s>\n",
				_DBG_PTR(pMad), pMad->payload.SIDR_REQ.RequestID,
				pMad->payload.SIDR_REQ.ServiceID, _DBG_PTR(FSTATUS_MSG(Status))));

	SendSIDR_RESP(pMad->payload.SIDR_REQ.ServiceID, 
				0, 0, 
				SRS_S_PROVIDER_REJECTED,
				NULL, 0,
				pMad->common.TransactionID,
				pMad->payload.SIDR_REQ.RequestID,
				pRemoteDgrm);
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// ProcessSidrResp()
//
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
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL. With ListLock held
//

FSTATUS
ProcessSidrResp(
	CM_MAD*	pMad,
	IBT_DGRM_ELEMENT* pRemoteDgrm
	)
{
	CM_CEP_OBJECT* pCEP=NULL;
	FSTATUS Status=FSUCCESS;
	CM_MAP_ITEM* pMapEntry;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessSidrResp);

	_DBG_INFO(("<mad 0x%p> SIDR_RESP received <reqid 0x%x sid 0x%"PRIx64">\n",					
					_DBG_PTR(pMad),
					pMad->payload.SIDR_RESP.RequestID,
					pMad->payload.SIDR_RESP.ServiceID));

	// Determine who received this CM MAD
	pMapEntry = CM_MapGet(&gCM->QueryMap, (uint64)pMad->payload.SIDR_RESP.RequestID);
	if (pMapEntry == CM_MapEnd(&gCM->QueryMap))
	{
		// Not found
		_DBG_WARNING(("<mad 0x%p> SIDR RESP dropped - no match found <reqid 0x%x sid 0x%"PRIx64">!\n",
				_DBG_PTR(pMad), pMad->payload.SIDR_RESP.RequestID,
				pMad->payload.SIDR_RESP.ServiceID));
		goto done;
	}
	pCEP = PARENT_STRUCT(pMapEntry, CM_CEP_OBJECT, MapEntry);
	ASSERT(pCEP->State == CMS_SIDR_REQ_SENT);

	Status = PrepareCepDgrm(pCEP);
	if (Status != FSUCCESS)
		goto fail;

	// Cancel the timer on QUERY
	CmTimerStop(pCEP, SIDR_REQ_TIMER);
	pCEP->RetryCount = 0;

	// Save the reply msg to the CEP obj.
	CmDgrmCopyAddrAndMad(pCEP->pDgrmElement, pMad, pRemoteDgrm);

	// Move from QUERY to IDLE
	CM_MapRemoveEntry(&gCM->QueryMap, &pCEP->MapEntry, QUERY_LIST, pCEP);
	// ClearEndPoint in CepToIdle is benign
	CepToIdle(pCEP);//CepSetState(pCEP, CMS_SIDR_RESP_RCVD);

	SetNotification(pCEP, CME_RCVD_SIDR_RESP); //EventSet(pCEP, CME_RCVD_SIDR_RESP);
	_DBG_INFO(("<mad 0x%p> SIDR RESP processed successfully <reqid 0x%x sid 0x%"PRIx64">!\n",
			_DBG_PTR(pMad), pMad->payload.SIDR_RESP.RequestID,
			pMad->payload.SIDR_RESP.ServiceID));

done:
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

fail:
	_DBG_WARNING(("<mad 0x%p> SIDR RESP dropped - unable to process it! <reqid 0x%x sid 0x%"PRIx64" %s>\n",
			_DBG_PTR(pMad), pMad->payload.SIDR_RESP.RequestID,
			pMad->payload.SIDR_RESP.ServiceID, _DBG_PTR(FSTATUS_MSG(Status))));
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;
}



//////////////////////////////////////////////////////////////////////////
// CmTimerCallback()
//
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
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
void
CmTimerCallback(
	IN void*	Context
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)Context;
	CM_MAP_ITEM* pMapEntry;
	uint32 timeout_ms;

	FSTATUS Status=FSUCCESS;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CmTimerCallback);
				
	//_DBG_INFO(("CmTimerCallback() - Curr thread %d\n", GetCurrentThreadId()))

	UNUSED_VAR(Status);

	SpinLockAcquire(&gCM->ListLock);

	// check the map to see if the CEP obj is on it. If the CEP obj is not here,
	// ignore this timer callback since the timer has already been canceled.
	pMapEntry = CM_MapGet(&gCM->TimerMap, (uint64)(uintn)pCEP);
	if (pMapEntry == CM_MapEnd(&gCM->TimerMap))
	{
		_DBG_WARNING(("<cep 0x%p> Timer has stopped but the callback got fired. Ignore it! <%s timestamp %"PRIu64"us elapsed %"PRIu64"us>!\n", 
						_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State)),
						GetTimeStamp(), CmGetElapsedTime(pCEP)));
		goto unlock;
	}

	if (TimerCallbackStopped(&pCEP->TimerObj))
		goto unlock; // stale timer callback

	// If we previously stop and restart the timer but the timer already fired, we will ignore it here.
	/**
	if (!CmTimerHasExpired(pCEP))
	{
		_DBG_WARNING(("<cep 0x%p> Timer has not expired. Ignore it! <%s timestamp %"PRIu64"us elapsed %"PRIu64"us>!\n", 
						_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State)),
						GetTimeStamp(), CmGetElapsedTime(pCEP)));
		goto unlock;
	}
	**/
	if (!CmTimerHasExpired(pCEP))
	{
		_DBG_WARNING(("<cep 0x%p> Timer has expired before its time! <%s interval %"PRIu64"us elapsed %"PRIu64"us>!\n", 
					_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State)),
					(uint64)pCEP->timeout_ms*1000, CmGetElapsedTime(pCEP)));
	}

	// Reset the timeout interval
	CmTimerClear(pCEP);

	switch (pCEP->State)
	{
	case CMS_REQ_SENT:
	case CMS_REP_WAIT:
		//
		// REQ timeout. Update the retry count and resend the REQ or
		// complete the REQ with timeout status
		//
		_DBG_INFO(("<cep 0x%p> <<<REQ Timer expired <lcid 0x%x %s %"PRIu64"us>\n", 
					_DBG_PTR(pCEP), pCEP->LocalCommID,
					_DBG_PTR(CmGetStateString(pCEP->State)), GetTimeStamp()));

		if (pCEP->RetryCount < pCEP->MaxCMRetries)
		{
			pCEP->RetryCount++;
			ResendREQ(pCEP);
		} else {
			_DBG_WARNING(("<cep 0x%p> Max REQ retries reached <lcid 0x%x retry cnt %d>\n", 
				_DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RetryCount));

			if (pCEP->bPeer && pCEP->RemoteEndPoint.CaGUID == 0)
			{
				CM_MapRemoveEntry(&gCM->ListenMap, &pCEP->MapEntry, LISTEN_LIST, pCEP);
			}

			// Send a reject indicating a timeout condition
			// TBD include LocalCaGuid in addinfo
			SendREJ(RC_TIMEOUT, 
					CM_REJECT_REPLY, 
					pCEP->TransactionID, 
					pCEP->LocalCommID, 
					pCEP->RemoteCommID, 
					pCEP->pDgrmElement);

			CepToIdle(pCEP);
			pCEP->RetryCount = 0;
			SetNotification(pCEP, CME_TIMEOUT_REQ);
		}

		break;

	case CMS_REP_SENT:
	case CMS_MRA_REP_RCVD:
		//
		// REP timeout. Update the retry count and resend the REP or
		// complete the REP with timeout status
		//
		_DBG_INFO(("<cep 0x%p> <<<REP Timer expired <lcid 0x%x %s %"PRIu64"us>\n", 
					_DBG_PTR(pCEP), pCEP->LocalCommID,
					_DBG_PTR(CmGetStateString(pCEP->State)), GetTimeStamp()));

		if (pCEP->RetryCount < pCEP->MaxCMRetries)
		{
			pCEP->RetryCount++;
			ResendREP(pCEP);
		} else {
			_DBG_WARNING(("<cep 0x%p> Max REP retries reached <lcid 0x%x cnt %d>!\n", 
						_DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RetryCount));

			// Send a reject indicating a timeout condition
			SendREJ(RC_TIMEOUT, 
					CM_REJECT_REQUEST, 
					pCEP->TransactionID, 
					pCEP->LocalCommID, 
					pCEP->RemoteCommID, 
					pCEP->pDgrmElement);

			CepToTimewait(pCEP);
			pCEP->RetryCount = 0;

			if (pCEP->bAsyncAccept && ! pCEP->bTimewaitCallback)
			{
				SetNotification(pCEP, CME_TIMEOUT_REP);	// callback
			} else {
				// for sync accept we will wakeup CmAccept
				// for Timewait, we set the event but defer the callback
				// until timewait timer expires
				EventSet(pCEP, CME_TIMEOUT_REP, FALSE);
			}
		}
		break;

	case CMS_LAP_SENT:
	case CMS_MRA_LAP_RCVD:
		//
		// LAP timeout. Update the retry count and resend the LAP or
		// complete the LAP with timeout status
		//
		_DBG_INFO(("<cep 0x%p> <<<LAP Timer expired <lcid 0x%x %s %"PRIu64"us>\n", 
					_DBG_PTR(pCEP), pCEP->LocalCommID,
					_DBG_PTR(CmGetStateString(pCEP->State)),
					GetTimeStamp()));

		if (pCEP->RetryCount < pCEP->MaxCMRetries)
		{
			pCEP->RetryCount++;

			// If dgrm is still in use (i.e. send pending), skip the retry
			// because pDgrmElement is still on SendQ
			// also it is likely the timeout is set too low
			if (!CmDgrmIsInUse(pCEP->pDgrmElement))
			{
				// The LAP is still valid in here
				ASSERT(GsiDgrmGetSendMad(pCEP->pDgrmElement)->common.AttributeID == MCLASS_ATTRIB_ID_LAP );

				_DBG_INFO(("<cep 0x%p> LAP resend <lcid 0x%x rcid 0x%x slid 0x%x dlid 0x%x>\n", 
						_DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RemoteCommID,
						pCEP->PrimaryPath.LocalLID, pCEP->PrimaryPath.RemoteLID));

				Status = CmDgrmSend(pCEP->pDgrmElement);
				if (FSUCCESS != Status)
				{
					// fall through and let timer retry the send later
					_DBG_WARN(("<cep 0x%p> DgrmSend() failed for LAP!!! <%s>\n",
								_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
					Status = FSUCCESS;
				} else {
					AtomicIncrementVoid(&gCM->Sent.Lap);
				}
			}

			CepSetState(pCEP, CMS_LAP_SENT);
			// Restart the timer
			timeout_ms = TimeoutMultToTimeInMs(pCEP->LocalCMTimeout);
			CmTimerStart(pCEP, timeout_ms, LAP_TIMER);
		} else {
			_DBG_WARNING(("<cep 0x%p> Max LAP retries reached <lcid 0x%x cnt %d>!\n", 
						_DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RetryCount));

			pCEP->bFailoverSupported = FALSE;
			pCEP->RetryCount = 0;
			CepSetState(pCEP, CMS_ESTABLISHED);
			SetNotification(pCEP, CME_TIMEOUT_LAP);	// callback
		}
		break;

	case CMS_DREQ_SENT:
		//
		// DREQ timeout. Update the retry count and resend the REQ or
		// complete the REQ with timeout status
		//
		_DBG_INFO(("<cep 0x%p> <<<DREQ Timer expired <lcid 0x%x CMS_DREQ_SENT %"PRIu64"us>\n", 
					_DBG_PTR(pCEP), pCEP->LocalCommID, GetTimeStamp()));

		if (pCEP->RetryCount < pCEP->MaxCMRetries)
		{
			pCEP->RetryCount++;

			// If dgrm is still in use (i.e. send pending), skip the retry
			// because pDgrmElement is still on SendQ
			// also it is likely the timeout is set too low
			if (!CmDgrmIsInUse(pCEP->pDgrmElement))
			{
				// The DREQ msg is still valid in here
				ASSERT(GsiDgrmGetSendMad(pCEP->pDgrmElement)->common.AttributeID == MCLASS_ATTRIB_ID_DREQ );

				_DBG_INFO(("<cep 0x%p> DREQ resend <lcid 0x%x rcid 0x%x slid 0x%x dlid 0x%x>\n", 
						_DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RemoteCommID,
						pCEP->PrimaryPath.LocalLID, pCEP->PrimaryPath.RemoteLID));

				Status = CmDgrmSend(pCEP->pDgrmElement);
				if (FSUCCESS != Status)
				{
					// fall through and let timer retry the send later
					_DBG_WARN(("<cep 0x%p> DgrmSend() failed for DREQ!!! <%s>\n",
								_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
					Status = FSUCCESS;
				} else {
					AtomicIncrementVoid(&gCM->Sent.Dreq);
				}
			}
			
			// Restart the timer
			timeout_ms = TimeoutMultToTimeInMs(pCEP->LocalCMTimeout);

			CmTimerStart(pCEP, timeout_ms, DREQ_TIMER);
		} else {
			_DBG_WARNING(("<cep 0x%p> Max DREQ retries reached <lcid 0x%x cnt %d>!\n", 
							_DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RetryCount));

			// We hit max retries or we are unable to resend the REQ
			CepToTimewait(pCEP);
			
			// for bTimewaitCallback, we will defer the FCM_DISCONNECTED
			// callback until timewait expires
			if (! pCEP->bTimewaitCallback)
			{
				SetNotification(pCEP, CME_TIMEOUT_DREQ);
			}
		}
		break;

	case CMS_TIMEWAIT:
		//
		// TIMEWAIT expires. Go back to IDLE state.
		//
		_DBG_INFO(("<cep 0x%p> <<<TIMEWAIT Timer expired <lcid 0x%x CMS_TIMEWAIT %"PRIu64"us>\n", 
					_DBG_PTR(pCEP), pCEP->LocalCommID, GetTimeStamp()));

		CepToIdle(pCEP);
			
		// Reset the appropriate fields

		pCEP->LocalCMTimeout = pCEP->RemoteCMTimeout= 0;
		pCEP->MaxCMRetries = 0;
		pCEP->RetryCount = 0;
		
		// Is this obj marked for destruction ?
		if (BitTest(pCEP->EventFlags, USRE_DESTROY))
		{
			// The check here handles the case when the user callback returns
			// and this routine gets
			// run because the timewait timer expired.
			// We let the CmSystemCallback() proceed with the destruction.
			if (AtomicRead(&pCEP->CallbackRefCnt) == 0)
			{
				DestroyCEP(pCEP);
				break;
			}
		}

		// set up for FCM_DISCONNECTED callback to tell user timewait is done
		// and QP/CEP can now be reused
		if (pCEP->bTimewaitCallback)
		{
			SetNotification(pCEP, CME_TIMEOUT_TIMEWAIT);
		}
		break;

	case CMS_SIDR_REQ_SENT:
		_DBG_INFO(("<cep 0x%p> <<<SIDR REQ Timer expired <lcid 0x%x CMS_SIDR_REQ_SENT %"PRIu64"us>\n", 
					_DBG_PTR(pCEP), pCEP->LocalCommID, GetTimeStamp()));
	
		if (pCEP->RetryCount < pCEP->MaxCMRetries)
		{
			pCEP->RetryCount++;
			// If dgrm is still in use (i.e. send pending), skip the timeout since it is
			// likely the timeout is set too low
			if (!CmDgrmIsInUse(pCEP->pDgrmElement))
			{
				// The SIDR_REQ msg is still valid in here
				ASSERT(GsiDgrmGetSendMad(pCEP->pDgrmElement)->common.AttributeID == MCLASS_ATTRIB_ID_SIDR_REQ );

				_DBG_INFO(("<cep 0x%p> SIDR_REQ resend <lcid 0x%x slid 0x%x dlid 0x%x>\n", 
							_DBG_PTR(pCEP), pCEP->LocalCommID, 
							pCEP->PrimaryPath.LocalLID, pCEP->PrimaryPath.RemoteLID));

				Status = CmDgrmSend(pCEP->pDgrmElement);
				if (FSUCCESS != Status)
				{
					// fall through and let timer retry the send later
					_DBG_WARN(("<cep 0x%p> DgrmSend() failed for SIDR_REQ!!! <%s>\n",
								_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
					Status = FSUCCESS;
				} else {
					AtomicIncrementVoid(&gCM->Sent.SidrReq);
				}
			}
			// Restart the timer
			CmTimerStart(pCEP, SidrTimeout(pCEP), SIDR_REQ_TIMER);
		} else {
			_DBG_WARNING(("<cep 0x%p> Max SIDR_REQ retries reached <lcid 0x%x cnt %d>!\n", 
							_DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RetryCount));

			// Move from QUERY to IDLE
			CM_MapRemoveEntry(&gCM->QueryMap, &pCEP->MapEntry, QUERY_LIST, pCEP);
			// ClearEndPoint in CepToIdle is benign
			CepToIdle(pCEP);
			pCEP->RetryCount = 0;
			SetNotification(pCEP, CME_TIMEOUT_SIDR_REQ); //EventSet(pCEP, CME_TIMEOUT_SIDR_REQ);
		}

		break;

	default:
		_DBG_WARNING(("<cep 0x%p> TIMEOUT not handled < timestamp %"PRIu64"us %s>!\n",
					_DBG_PTR(pCEP), GetTimeStamp(),
					_DBG_PTR(CmGetStateString(pCEP->State))));

		break;
	} // switch()

unlock:
	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return;

} // CmTimerCallback()


//////////////////////////////////////////////////////////////////////////
// CmSystemCallback()
//
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
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
void
CmSystemCallback(void *Key, void* Context)
{
	CM_CEP_OBJECT*	pCEP=(CM_CEP_OBJECT*)Context;

	PFN_CM_CALLBACK	pfnUserCB=NULL;

	CM_CONN_INFO	*ConnInfo = NULL;
	FSTATUS			Status=FSUCCESS;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CmSystemCallback);


	_DBG_INFO(("<cep 0x%p> System callback item dequeued. <%s cb item 0x%p cb refcnt %d>\n",
				_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State)),
				_DBG_PTR(pCEP->pCallbackItem), AtomicRead(&pCEP->CallbackRefCnt)));

	SpinLockAcquire(&gCM->ListLock);

	if (BitTest(pCEP->EventFlags, USRE_DESTROY))
	{
		_DBG_WARNING(("<cep 0x%p> Object marked for destruction - skip callback <%s>\n",
					_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State))));

		pfnUserCB = NULL;
	} else {
		ConnInfo = (CM_CONN_INFO*)MemoryAllocateAndClear(sizeof(*ConnInfo), FALSE, CM_MEM_TAG);
		if (ConnInfo)
		{
			// Get the connection info
			Status = GetConnInfo(pCEP, ConnInfo);

			if (Status != FSUCCESS)
			{
				// Skip the callback since GetConnInfo() failed (i.e. no status to report)
				_DBG_WARNING(("<cep 0x%p> No status to report - skip callback <%s>\n",
						_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State))));

				pfnUserCB = NULL;
			} else {
				// Save the callback func here before we release the lock
				pfnUserCB = pCEP->pfnUserCallback;
			}
		} else {
			// Skip the callback since GetConnInfo() failed (i.e. no status to report)
			_DBG_WARN(("<cep 0x%p> No status to report - skip callback <%s> - no memory\n",
					_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State))));

			pfnUserCB = NULL;
		}
	}
	
	SpinLockRelease(&gCM->ListLock);

	// there is a race here, since we have released the Lock, it is possible
	// for user to Destroy the CEP here and race with this callback, in which
	// case the callback hits the user after CmDestroyCEP has returned
	// to the user.  We can't hold the lock during the callback since that
	// would prevent user from making Cm calls in their callback.
	// however we cannot prevent CmDestroyCEP from being called before or during
	// user callback
	if (pfnUserCB)
	{
		_DBG_INFO(("<cep 0x%p> Calling user callback...<%s %s context 0x%p>\n",
				_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State)),
				_DBG_PTR(FSTATUS_MSG(ConnInfo->Status)), _DBG_PTR(pCEP->UserContext)));

		pfnUserCB(pCEP, ConnInfo, pCEP->UserContext);

		_DBG_INFO(("<cep 0x%p> Returning from user callback <%s>\n",
				_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State))));
	} else {
		_DBG_WARNING(("<cep 0x%p> User callback function is NULL. Skip callback\n", _DBG_PTR(pCEP)));
	}
	if (ConnInfo)
	{
		MemoryDeallocate(ConnInfo);
	}

	CmDoneCallback(pCEP);
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

} // CmSystemCallback()

// decrement counters to reflect we completed present callback
// then check to see if there are more queued callbacks on the CEP
// if so schedule them
void
CmDoneCallback(CM_CEP_OBJECT*	pCEP)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessQueuedCallbacks);

	// Determine if there is another callback pending
	SpinLockAcquire(&gCM->ListLock);
	
	// Decrement the ref cnt to indicate no callback in progress
	AtomicDecrementVoid(&pCEP->CallbackRefCnt);
	AtomicDecrementVoid(&gCM->TotalCallbackRefCnt);

	// Is this obj marked for destruction ?
	if (BitTest(pCEP->EventFlags, USRE_DESTROY))
	{
		// Allow the caller to know that there is no callback event pending during unload process
		if (pCEP->pCallbackEvent)
			EventTrigger(pCEP->pCallbackEvent);

		AtomicWrite(&pCEP->CallbackRefCnt, 0);	// we will start no more
		if (pCEP->State == CMS_IDLE)
		{
			DestroyCEP(pCEP);
		} else {
			_DBG_WARNING(("<cep 0x%p> Cannot destroy CEP obj in this state <%s cb refcnt %d>\n", 
							_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State)),
							AtomicRead(&pCEP->CallbackRefCnt)));
		}
	} else {
		// If there are still outstanding callback, queue the callback item
		if (AtomicRead(&pCEP->CallbackRefCnt))
		{
			_DBG_INFO(("<cep 0x%p> Queuing callback item <cb item 0x%p cb refcnt %d>.\n", 
						_DBG_PTR(pCEP), _DBG_PTR(pCEP->pCallbackItem),
						AtomicRead(&pCEP->CallbackRefCnt)));

			EventClear(&gCM->CallbackEventObj);

			if (pCEP->pCallbackEvent)
				EventClear(pCEP->pCallbackEvent);

			AtomicIncrementVoid(&gCM->TotalCallbackRefCnt);

			// Queue the work item for user callback
			SysCallbackQueue(pCEP->pCallbackItem, CmSystemCallback, pCEP, FALSE );
		} else {
			_DBG_INFO(("<cep 0x%p> No callback pending <cb item 0x%p cb refcnt %d>\n", 
							_DBG_PTR(pCEP), _DBG_PTR(pCEP->pCallbackItem),
							AtomicRead(&pCEP->CallbackRefCnt)));

			// Allow the caller to know that there is no callback event pending during unload process
			if (pCEP->pCallbackEvent)
				EventTrigger(pCEP->pCallbackEvent);
		}
	}

	if (AtomicRead(&gCM->TotalCallbackRefCnt) == 0)
	{
		EventTrigger(&gCM->CallbackEventObj);
	}

	SpinLockRelease(&gCM->ListLock);
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}

// Make sure pCEP->pDgrmElement is available to retain a new message
// If its still busy (send in progress), we release the Dgrm (Send completion
// will free it) and get a new Dgrm for the CEP
// We do a GsiDgrmAddrCopy, however most callers of this
// function will copy over that address, however a few callers require the
// address to be copied.  So its simpler to do the extra
// copy to reduce duplicate code.
FSTATUS
PrepareCepDgrm(CM_CEP_OBJECT* pCEP)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, PrepareCepDgrm);

	if (CmDgrmIsInUse(pCEP->pDgrmElement))
	{
		IBT_DGRM_ELEMENT* pNewDgrm;

		_DBG_WARNING(("<cep 0x%p> Dgrm is in use. Get a new one! <dgrm 0x%p>\n",
					_DBG_PTR(pCEP), _DBG_PTR(pCEP->pDgrmElement)));
		pNewDgrm = CmDgrmGet();
		if (!pNewDgrm)
		{
			/* CmDgrmGet already output an error, caller will provide any further information */
			return FINSUFFICIENT_RESOURCES;
		}
		CmDgrmSetReleaseFlag(pCEP->pDgrmElement);
		GsiDgrmAddrCopy(pNewDgrm, pCEP->pDgrmElement);
		pCEP->pDgrmElement = pNewDgrm;
	}
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return FSUCCESS;
}

/* log messages to indicate why a CmDgrmGet Failed */
void
CmDgrmGetFailed(void)
{
	uint32 savelast;
	static uint32 lasttime = 0;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CmDgrmGetFailed);
	/* to avoid flooding the log when huge app startup fails, we
	 * limit output of this message to 1 per second
	 */
	/* there is a possible race here, but on most systems assignment of
	 * uint32 is atomic, and worse case we output some extra log messages
	 * in addition the CM global lock should protect us
	 */
	savelast = lasttime;
	lasttime = GetTimeStampSec();
	if (savelast != lasttime) {
		uint32 total = iba_gsi_dgrm_pool_total(gCM->GSADgrmPoolHandle);
		if (total < gCmParams.MaxDgrms) {
			_DBG_ERROR(("CmDgrmGet: Temporarily Out of CM Dgrms: Allocated: %u CmMinFreeDgrms: %u CmDgrmIncrement: %u CmDgrmPreallocDgrms: %u\nThis error can be caused by excessive application connections or inadequate configuration of above values\n",
				total, gCmParams.MinFreeDgrms, gCmParams.DgrmIncrement,
				gCmParams.PreallocDgrms));
		} else {
			_DBG_ERROR(("CmDgrmGet: Out of CM Dgrms, reached CmMaxDgrms limit of %u\nThis error can be caused by excessive application connections or inadequate configuration of CmMaxDgrms\n", gCmParams.MaxDgrms));
		}
	}
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}

//////////////////////////////////////////////////////////////////////////
// SendREJ()
//
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
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
SendREJ(CM_REJECTION_CODE		Reason, 
		   CMM_REJ_MSGTYPE		MsgRejected,
		   uint64				TransactionID,
		   uint32				LocalCommID,
		   uint32				RemoteCommID,
		   IBT_DGRM_ELEMENT*	pRemoteDgrm)
{
	FSTATUS Status=FSUCCESS;
	IBT_DGRM_ELEMENT* pDgrmElement=NULL;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SendREJ);

	// Send a reject
	pDgrmElement = CmDgrmGet();
	if (!pDgrmElement)
	{
		_DBG_WARNING(("failed to send REJ: Out of CM Dgrms!\n"));
		Status = FINSUFFICIENT_RESOURCES;
		goto fail;
	}
			
	GsiDgrmAddrCopy(pDgrmElement, pRemoteDgrm);

	// Format the REJ msg
	FormatREJ((CM_MAD*)GsiDgrmGetSendMad(pDgrmElement),
				(uint8)MsgRejected,
				(uint16)Reason,
				NULL, 0,
				NULL, 0,
				TransactionID,
				LocalCommID,
				RemoteCommID);
	
	// In SendComplete(), we return the dgrm to the pool
	Status = CmDgrmSendAndRelease(pDgrmElement);
	if (Status == FSUCCESS)
	{
		if (MsgRejected == CM_REJECT_REQUEST)
			AtomicIncrementVoid(&gCM->Sent.RejReq);
		else
			AtomicIncrementVoid(&gCM->Sent.RejRep);
#ifdef ICS_LOGGING
		_DBG_INFO(("<dgrm 0x%p> REJ sent rejecting %s <lcid 0x%x rcid 0x%x dlid 0x%x >.\n", 
					_DBG_PTR(pDgrmElement),
					_DBG_PTR((MsgRejected == CM_REJECT_REQUEST)?"REQ":"REP"),
					LocalCommID, RemoteCommID, pRemoteDgrm->RemoteLID));

		_DBG_INFO(("<dgrm 0x%p> REJ sent rejecting %s reason %d %s\n",
				   _DBG_PTR(pDgrmElement),
				   _DBG_PTR((MsgRejected == CM_REJECT_REQUEST)?"REQ":"REP"),
				   Reason, _DBG_PTR(FSTATUS_MSG(Status))));
#else
		_DBG_INFO(("<dgrm 0x%p> REJ sent rejecting %s <lcid 0x%x rcid 0x%x dlid 0x%x reason %d %s>.\n", 
					_DBG_PTR(pDgrmElement),
					_DBG_PTR((MsgRejected == CM_REJECT_REQUEST)?"REQ":"REP"),
					LocalCommID, RemoteCommID, pRemoteDgrm->RemoteLID,
					Reason, _DBG_PTR(FSTATUS_MSG(Status))));
#endif		
		
	} else {

		_DBG_WARN(("<dgrm 0x%p> DgrmSend() failed!!! Unable to send REJ <lcid 0x%x rcid 0x%x dlid 0x%x reason %d %s>.\n", 
					_DBG_PTR(pDgrmElement), LocalCommID, RemoteCommID,
					pRemoteDgrm->RemoteLID, Reason, _DBG_PTR(FSTATUS_MSG(Status))));

		CmDgrmPut(pDgrmElement);
	}

	// In SendComplete(), we return the dgrm to the pool
fail:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	
	return Status;

} // SendREJ()
 

//////////////////////////////////////////////////////////////////////////
// SendMRA()
//
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
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
SendMRA(CMM_MRA_MSGTYPE		MsgMraed,
			uint8				ServiceTimeout,
			uint64				TransactionID,
			uint32				LocalCommID,
			uint32				RemoteCommID,
			IBT_DGRM_ELEMENT*	pRemoteDgrm)
{
	FSTATUS Status=FSUCCESS;
	IBT_DGRM_ELEMENT* pDgrmElement=NULL;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SendMRA);

	// Send a mra
	pDgrmElement = CmDgrmGet();
	if (!pDgrmElement)
	{
		_DBG_WARNING(("failed to send MRA: Out of CM Dgrms!\n"));
		Status = FINSUFFICIENT_RESOURCES;
		goto fail;
	}
		
	GsiDgrmAddrCopy(pDgrmElement, pRemoteDgrm);
	
	// Format the MRA msg
	FormatMRA((CM_MAD*)GsiDgrmGetSendMad(pDgrmElement),
				(uint8)MsgMraed,
				(uint8)ServiceTimeout,
				NULL, 0,
				TransactionID,
				LocalCommID,
				RemoteCommID);
	
	// In SendComplete(), we return the dgrm to the pool
	Status = CmDgrmSendAndRelease(pDgrmElement);
	if (Status == FSUCCESS)
	{
		if (MsgMraed == CM_MRA_REQUEST)
			AtomicIncrementVoid(&gCM->Sent.MraReq);
		else if (MsgMraed == CM_MRA_REPLY)
			AtomicIncrementVoid(&gCM->Sent.MraRep);
		else
			AtomicIncrementVoid(&gCM->Sent.MraLap);
#ifdef ICS_LOGGING
		_DBG_INFO(("<dgrm 0x%p> MRA sent on %s <lcid 0x%x rcid 0x%x dlid 0x%x service timeout %d >.\n", 
					_DBG_PTR(pDgrmElement),
					_DBG_PTR((MsgMraed == CM_MRA_REQUEST)?"REQ":"REP"),
					LocalCommID, RemoteCommID, pRemoteDgrm->RemoteLID, ServiceTimeout));
		_DBG_INFO(("<dgrm 0x%p> MRA sent on %s <%s>.\n", 
					_DBG_PTR(pDgrmElement),
					_DBG_PTR((MsgMraed == CM_MRA_REQUEST)?"REQ":"REP"),
					_DBG_PTR(FSTATUS_MSG(Status))));

#else
		_DBG_INFO(("<dgrm 0x%p> MRA sent on %s <lcid 0x%x rcid 0x%x dlid 0x%x service timeout %d %s>.\n", 
					_DBG_PTR(pDgrmElement),
					_DBG_PTR((MsgMraed == CM_MRA_REQUEST)?"REQ":"REP"),
					LocalCommID, RemoteCommID, pRemoteDgrm->RemoteLID,
					ServiceTimeout, _DBG_PTR(FSTATUS_MSG(Status))));
#endif
	} else {
		_DBG_WARN(("<dgrm 0x%p> DgrmSend() failed!!! Unable to send MRA <lcid 0x%x rcid 0x%x dlid 0x%x service timeout %d %s>.\n", 
					_DBG_PTR(pDgrmElement), LocalCommID, RemoteCommID,
					pRemoteDgrm->RemoteLID, ServiceTimeout, _DBG_PTR(FSTATUS_MSG(Status))));

		CmDgrmPut(pDgrmElement);
	}

	// In SendComplete(), we return the dgrm to the pool
fail:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	
	return Status;
} // SendMRA()

//////////////////////////////////////////////////////////////////////////
// ResendREQ()
//
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
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
ResendREQ(IN CM_CEP_OBJECT*	pCEP)
{
	FSTATUS Status = FSUCCESS;
	uint32 timeout_ms;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ResendREQ);

	// If dgrm is still in use (i.e. send pending), skip the retry
	// because pDgrmElement is still on SendQ
	// If this is not active Active Peer, it's likely the timeout is set too low
	// We will let the timer try again later
	if (!CmDgrmIsInUse(pCEP->pDgrmElement))
	{
		// The REQ is still valid in here
		ASSERT(GsiDgrmGetSendMad(pCEP->pDgrmElement)->common.AttributeID == MCLASS_ATTRIB_ID_REQ );

		_DBG_WARNING(("<cep 0x%p> REQ resend <lcid 0x%x retry cnt %d>!\n", 
						_DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RetryCount));

		Status = CmDgrmSend(pCEP->pDgrmElement);
		if (FSUCCESS != Status)
		{
			// fall through and let timer retry the send later
			_DBG_WARN(("<cep 0x%p> DgrmSend() failed for REQ!!! <%s>\n",
					_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
			Status = FSUCCESS;
		} else {
			AtomicIncrementVoid(&gCM->Sent.Req);
		}
	}

	CepSetState(pCEP, CMS_REQ_SENT);

	// Restart the timer
	timeout_ms = TimeoutMultToTimeInMs(pCEP->LocalCMTimeout);
	CmTimerStart(pCEP, timeout_ms, REQ_TIMER);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;
}


//////////////////////////////////////////////////////////////////////////
// ResendREP()
//
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
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
ResendREP(IN CM_CEP_OBJECT*	pCEP)
{
	FSTATUS Status = FSUCCESS;
	uint32 timeout_ms;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ResendREP);

	// If dgrm is still in use (i.e. send pending), skip the retry
	// because pDgrmElement is still on SendQ
	// also it is likely the timeout is set too low
	if (!CmDgrmIsInUse(pCEP->pDgrmElement))
	{
		// The REP is still valid in here
		ASSERT(GsiDgrmGetSendMad(pCEP->pDgrmElement)->common.AttributeID == MCLASS_ATTRIB_ID_REP );

		_DBG_INFO(("<cep 0x%p> REP resend <lcid 0x%x rcid 0x%x slid 0x%x dlid 0x%x>\n", 
					_DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RemoteCommID,
					pCEP->PrimaryPath.LocalLID, pCEP->PrimaryPath.RemoteLID));

		Status = CmDgrmSend(pCEP->pDgrmElement);
		if (FSUCCESS != Status)
		{
			// fall through and let timer retry the send later
			_DBG_WARN(("<cep 0x%p> DgrmSend() failed for REP!!! <%s>\n",
						_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
			Status = FSUCCESS;
		} else {
			AtomicIncrementVoid(&gCM->Sent.Rep);
		}
	}

	CepSetState(pCEP, CMS_REP_SENT);
	// Restart the timer
	timeout_ms = TimeoutMultToTimeInMs(pCEP->LocalCMTimeout);
	CmTimerStart(pCEP, timeout_ms, REP_TIMER);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// SendRTU()
//
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
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
SendRTU(uint64				TransactionID,
		   uint32				LocalCommID,
		   uint32				RemoteCommID,
		   IBT_DGRM_ELEMENT*	pRemoteDgrm)
{
	FSTATUS Status=FSUCCESS;
	IBT_DGRM_ELEMENT* pDgrmElement=NULL;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SendRTU);

	pDgrmElement = CmDgrmGet();
	if (!pDgrmElement)
	{
		_DBG_WARNING(("failed to send RTU: Out of CM Dgrms!\n"));
		Status = FINSUFFICIENT_RESOURCES;
		goto fail;
	}
			
	GsiDgrmAddrCopy(pDgrmElement, pRemoteDgrm);

	// RTU with no private data
	FormatRTU((CM_MAD*)GsiDgrmGetSendMad(pDgrmElement), 
				NULL,
				0,
				TransactionID,
				LocalCommID,
				RemoteCommID);
	
	// In SendComplete(), we return the dgrm to the pool
	Status = CmDgrmSendAndRelease(pDgrmElement);
	if (Status == FSUCCESS)
	{
		AtomicIncrementVoid(&gCM->Sent.Rtu);
		_DBG_INFO(("<dgrm 0x%p> RTU sent <lcid 0x%x rcid 0x%x dlid 0x%x >.\n", 
					_DBG_PTR(pDgrmElement), 
					LocalCommID, RemoteCommID, pRemoteDgrm->RemoteLID));
	} else {
		_DBG_ERROR(("<dgrm 0x%p> DgrmSend() failed!!! Unable to send RTU <lcid 0x%x rcid 0x%x dlid 0x%x %s>.\n", 
					_DBG_PTR(pDgrmElement), LocalCommID, RemoteCommID,
					pRemoteDgrm->RemoteLID, _DBG_PTR(FSTATUS_MSG(Status))));

		CmDgrmPut(pDgrmElement);
	}

	// In SendComplete(), we return the dgrm to the pool
fail:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	
	return Status;

} // SendRTU()

//////////////////////////////////////////////////////////////////////////
// SendAPR()
// Send an APR to reject an inbound LAP
//
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
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
SendAPR(CM_APR_STATUS		APStatus, 
		   uint64				TransactionID,
		   uint32				LocalCommID,
		   uint32				RemoteCommID,
		   IBT_DGRM_ELEMENT*	pRemoteDgrm)
{
	FSTATUS Status=FSUCCESS;
	IBT_DGRM_ELEMENT* pDgrmElement=NULL;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SendAPR);

	// Send an APR
	pDgrmElement = CmDgrmGet();
	if (!pDgrmElement)
	{
		_DBG_WARNING(("failed to send APR: Out of CM Dgrms!\n"));
		Status = FINSUFFICIENT_RESOURCES;
		goto fail;
	}
			
	GsiDgrmAddrCopy(pDgrmElement, pRemoteDgrm);

	// Format the APR msg
	FormatAPR((CM_MAD*)GsiDgrmGetSendMad(pDgrmElement),
				APStatus,
				NULL, 0,
				NULL, 0,
				TransactionID,
				LocalCommID,
				RemoteCommID);
	
	// In SendComplete(), we return the dgrm to the pool
	Status = CmDgrmSendAndRelease(pDgrmElement);
	if (Status == FSUCCESS)
	{
		if (APStatus == APS_PATH_LOADED)
			AtomicIncrementVoid(&gCM->Sent.AprAcc);
		else
			AtomicIncrementVoid(&gCM->Sent.AprRej);
		_DBG_INFO(("<dgrm 0x%p> APR sent <lcid 0x%x rcid 0x%x dlid 0x%x APStatus %d %s>.\n", 
					_DBG_PTR(pDgrmElement),
					LocalCommID, RemoteCommID, pRemoteDgrm->RemoteLID,
					APStatus, _DBG_PTR(FSTATUS_MSG(Status))));
		
	} else {

		_DBG_ERROR(("<dgrm 0x%p> DgrmSend() failed!!! Unable to send APR <lcid 0x%x rcid 0x%x dlid 0x%x APStatus %d %s>.\n", 
					_DBG_PTR(pDgrmElement), LocalCommID, RemoteCommID,
					pRemoteDgrm->RemoteLID, APStatus, _DBG_PTR(FSTATUS_MSG(Status))));

		CmDgrmPut(pDgrmElement);
	}

	// In SendComplete(), we return the dgrm to the pool
fail:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	
	return Status;

} // SendAPR()
 
//////////////////////////////////////////////////////////////////////////
// SendSIDR_RESP()
//
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
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
SendSIDR_RESP(IN uint64				SID,
				 uint32				QPN,
				 uint32				QKey,
				 uint32				SIDRStatus,
				 const uint8*		pPrivateData,
				 uint32				DataLen,
				 uint64				TransactionID,
				 uint32				RequestID,
				 IBT_DGRM_ELEMENT*	pRemoteDgrm)
{
	FSTATUS Status=FSUCCESS;
	IBT_DGRM_ELEMENT* pDgrmElement=NULL;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SendSIDR_RESP);

	// Get a MAD datagram
	pDgrmElement = CmDgrmGet();
	if (!pDgrmElement)
	{
		_DBG_WARNING(("failed to send SIDR_RESP: Out of CM Dgrms!\n"));
		Status = FINSUFFICIENT_RESOURCES;
		goto fail;
	}
				
	// Setup the dgrm's dest addr
	GsiDgrmAddrCopy(pDgrmElement, pRemoteDgrm);

	// Format the REJ msg
	FormatSIDR_RESP((CM_MAD*)GsiDgrmGetSendMad(pDgrmElement),
					SID,
					QPN,
					QKey,
					SIDRStatus,
					pPrivateData,
					DataLen,
					TransactionID,
					RequestID);
	
	// In SendComplete(), we return the dgrm to the pool
	Status = CmDgrmSendAndRelease(pDgrmElement);
	if (Status == FSUCCESS)
	{
		AtomicIncrementVoid(&gCM->Sent.SidrResp);
#ifdef ICS_LOGGING
	  _DBG_INFO(("<dgrm 0x%p> SIDR_RESP sent <req id 0x%x, sid 0x%"PRIx64", dlid 0x%x, QPN %u, QKey 0x%x >.\n", 
					_DBG_PTR(pDgrmElement), RequestID, SID, pRemoteDgrm->RemoteLID, QPN, QKey));
	  _DBG_INFO(("<dgrm 0x%p> SIDR_RESP sent <req id 0x%x,  sidr status %u %s>.\n", 
					_DBG_PTR(pDgrmElement), RequestID,  SIDRStatus, _DBG_PTR(FSTATUS_MSG(Status))));

#else
		_DBG_INFO(("<dgrm 0x%p> SIDR_RESP sent <req id 0x%x, sid 0x%"PRIx64", dlid 0x%x, QPN %u, QKey 0x%x sidr status %u %s>.\n", 
					_DBG_PTR(pDgrmElement), RequestID, SID, pRemoteDgrm->RemoteLID, QPN,
					QKey, SIDRStatus, _DBG_PTR(FSTATUS_MSG(Status))));
#endif

	} else {
#ifdef ICS_LOGGING
		_DBG_WARN(("<dgrm 0x%p> DgrmSend() failed!!! Unable to send SIDR_RESP <req id 0x%x, sid 0x%"PRIx64", dlid 0x%x, QPN %u, QKey 0x%x >.\n", 
					_DBG_PTR(pDgrmElement), RequestID, SID, pRemoteDgrm->RemoteLID, QPN, QKey ));
		_DBG_WARN(("<dgrm 0x%p> DgrmSend() failed!!! Unable to send SIDR_RESP <req id 0x%x, sidr status %u %s>.\n", 
					_DBG_PTR(pDgrmElement), RequestID,  SIDRStatus, _DBG_PTR(FSTATUS_MSG(Status))));
	  
#else
		_DBG_WARN(("<dgrm 0x%p> DgrmSend() failed!!! Unable to send SIDR_RESP <req id 0x%x, sid 0x%"PRIx64", dlid 0x%x, QPN %u, QKey 0x%x sidr status %u %s>.\n", 
					_DBG_PTR(pDgrmElement), RequestID, SID, pRemoteDgrm->RemoteLID,
					QPN, QKey, SIDRStatus, _DBG_PTR(FSTATUS_MSG(Status))));
#endif
		CmDgrmPut(pDgrmElement);
	}

	// In SendComplete(), we return the dgrm to the pool
fail:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	
	return Status;

} // SendSIDR_RESP()

//////////////////////////////////////////////////////////////////////////
// SendDREQ()
//
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
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
SendDREQ(uint32 QPN,
			uint64 TransactionID,
			uint32 LocalCommID,
		   uint32 RemoteCommID,
		   IBT_DGRM_ELEMENT*	pRemoteDgrm)
{
	FSTATUS Status=FSUCCESS;
	IBT_DGRM_ELEMENT* pDgrmElement=NULL;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SendDREQ);

	// Get a dgrm
	pDgrmElement = CmDgrmGet();
	if (!pDgrmElement)
	{
		_DBG_WARNING(("failed to send DREQ: Out of CM Dgrms!\n"));
		Status = FINSUFFICIENT_RESOURCES;
		goto fail;
	}
				
	// Setup the dgrm's dest addr
	GsiDgrmAddrCopy(pDgrmElement, pRemoteDgrm);

	// Format the DREQ
	FormatDREQ((CM_MAD*)GsiDgrmGetSendMad(pDgrmElement),
				QPN,
				NULL, 0,
				TransactionID,
				LocalCommID,
				RemoteCommID);
	
	// Release back to the pool when send complete.
	Status = CmDgrmSendAndRelease(pDgrmElement);
	if (Status == FSUCCESS)
	{
		AtomicIncrementVoid(&gCM->Sent.Dreq);
		_DBG_INFO(("<dgrm 0x%p> DREQ sent <lcid 0x%x rcid 0x%x dlid 0x%x>.\n", 
					_DBG_PTR(pDgrmElement), LocalCommID, RemoteCommID, pRemoteDgrm->RemoteLID));
	} else {
		_DBG_WARN(("<dgrm 0x%p> DgrmSend() failed!!! Unable to send DREQ <lcid 0x%x rcid 0x%x dlid 0x%x>.\n", 
					_DBG_PTR(pDgrmElement), LocalCommID, RemoteCommID, pRemoteDgrm->RemoteLID));

		CmDgrmPut(pDgrmElement);
	}

	// In SendComplete(), we return the dgrm to the pool
fail:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	
	return Status;

} // SendDREQ()


//////////////////////////////////////////////////////////////////////////
// SendDREP()
//
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
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
SendDREP( uint64 TransactionID,
		   uint32 LocalCommID,
		   uint32 RemoteCommID,
		   IBT_DGRM_ELEMENT*	pRemoteDgrm)
{
	FSTATUS Status=FSUCCESS;
	IBT_DGRM_ELEMENT* pDgrmElement=NULL;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SendDREP);

	// Get a dgrm
	pDgrmElement = CmDgrmGet();
	if (!pDgrmElement)
	{
		_DBG_WARNING(("failed to send DREP: Out of CM Dgrms!\n"));
		Status = FINSUFFICIENT_RESOURCES;
		goto fail;
	}
				
	// Setup the dgrm's dest addr
	GsiDgrmAddrCopy(pDgrmElement, pRemoteDgrm);

	// Format the DREP
	FormatDREP((CM_MAD*)GsiDgrmGetSendMad(pDgrmElement),
				NULL, 0,
				TransactionID,
				LocalCommID,
				RemoteCommID);
	
	// Release back to the pool when send complete.
	Status = CmDgrmSendAndRelease(pDgrmElement);
	if (Status == FSUCCESS)
	{
		AtomicIncrementVoid(&gCM->Sent.Drep);
		_DBG_INFO(("<dgrm 0x%p> DREP sent <lcid 0x%x rcid 0x%x dlid 0x%x>.\n", 
					_DBG_PTR(pDgrmElement), LocalCommID, RemoteCommID, pRemoteDgrm->RemoteLID));
	} else {
		_DBG_WARN(("<dgrm 0x%p> DgrmSend() failed!!! Unable to send DREP <lcid 0x%x rcid 0x%x dlid 0x%x>.\n", 
					_DBG_PTR(pDgrmElement), LocalCommID, RemoteCommID, pRemoteDgrm->RemoteLID));

		CmDgrmPut(pDgrmElement);
	}

	// In SendComplete(), we return the dgrm to the pool
fail:
	_DBG_FSTATUS_RETURN_LVL(_DBG_LVL_FUNC_TRACE,Status);
	return Status;

} // SendDREP()


//////////////////////////////////////////////////////////////////////////
// SetNotification()
//
// Notify consumer of an event.  If there is a user callback function, we
// will trigger a system callback.  If there is no user callback function we
// will Trigger the event and allow the user to wait for it.
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
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
boolean
SetNotification(CM_CEP_OBJECT* pCEP, uint32 EventType)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SetNotification);

	// If this obj is marked for destruction, no need to notify
	if (BitTest(pCEP->EventFlags, USRE_DESTROY))
	{
		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return FALSE;
	}

	if (pCEP->pfnUserCallback)
	{
		// kernel mode consumer w/callback
		BitSet(pCEP->EventFlags, EventType);
			
		// Increment the ref cnt so that we dont delete this obj
		AtomicIncrementVoid(&pCEP->CallbackRefCnt);

		// Allow only 1 callback outstanding per CEP obj. This also serialize the
		// callback notification per CEP
		if (AtomicRead(&pCEP->CallbackRefCnt) == 1)
		{
			EventClear(&gCM->CallbackEventObj);

			if (pCEP->pCallbackEvent)
				EventClear(pCEP->pCallbackEvent);

			AtomicIncrementVoid(&gCM->TotalCallbackRefCnt);

			// Queue the callback item for user callback
			SysCallbackQueue(pCEP->pCallbackItem, CmSystemCallback, pCEP, FALSE );
						
			_DBG_INFO(("<cep 0x%p> Queuing callback item <cb item 0x%p cb refcnt %u>.\n", 
						_DBG_PTR(pCEP), _DBG_PTR(pCEP->pCallbackItem),
						AtomicRead(&pCEP->CallbackRefCnt)));
		} else {
			_DBG_INFO(("<cep 0x%p> There is already a callback outstanding <cb item 0x%p cb refcnt %u>.\n", 
						_DBG_PTR(pCEP), _DBG_PTR(pCEP->pCallbackItem),
						AtomicRead(&pCEP->CallbackRefCnt)));
		}
	}
	else
	{
		// user mode consumer or non-callback model
		EventSet(pCEP, EventType, TRUE);
	}

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return TRUE;

} // SetNotification()

// ListenMap Key Compare functions
// Three functions are provided:
//
// CepListenAddrCompare - is used to insert cep entries into the ListenMap and
// is the primary key_compare function for the ListenMap
//
// CepReqAddrCompare - is used to search the ListenMap as part of processing
// an inbound REQ
//
// CepSidrReqAddrCompare - is used to search the ListenMap as part of
// processing an inbound SIDR_REQ
//
// To provide the maximum flexibilty, the key for a CEP bound address is
// sophisticated and allows wildcarded/optional fields.  This allows
// a listener to simply bind for all traffic of a given SID or to refine the
// scope by binding for traffic to/from specific addresses, or specific
// private data.  The QPN/EECN/CaGUID aspect is used to allow multiple
// outbound Peer Connects to still be considered unique.
//
// The result of this approach is very flexible CM bind.  The same SID
// can be used on different ports or between different node pairs for
// completely different meanings.  However a SID used between a given
// pair of nodes must be used for a single model (Listen, Peer, Sidr)
// In addition for Peer connects, each connect must have a unique
// QPN/EECN/CaGUID.
//
// Comparision allows for wildcarding in all but SID
// A value of 0 is a wildcard.  See ib_helper.h:WildcardGidCompare for
// the rules of GID comparision, which are more involved due to multiple Gid
// formats
//
//                                          Field is Used by models as follows:
// Coallating order is:                  Listen     Peer Connect   Sidr Register
// SID                                     Y             Y              Y
// local GID                             option          Y         future option
// local LID                             option          Y         future option
// QPN                                  wildcard         Y           wildcard
// EECN                                 wildcard         Y           wildcard
// CaGUID                               wildcard         Y           wildcard
// remote GID                            option          Y         future option
// remote LID                            option          Y         future option
// private data discriminator length     option        option         option
// private data discriminator value      option        option         option
//
// if bPeer is 0 for either CEP, the QPN, EECN and CaGUID are treated as a match
//
// FUTURE: add a sid masking option so can easily listen on a group
// of SIDs with 1 listen (such as if low bits of sid have a private meaning)
//
// FUTURE: add a pkey option so can easily listen on a partition
//
// FUTURE: for SIDR to support GID/LID they will have to come from the LRH
// and GRH headers to the CM mad.  local GID and lid could be used to merely
// select the local port number


// A qmap key_compare function to compare the bound address for
// two listener, SIDR or Peer Connect CEPs
//
// key1 - CEP1 pointer
// key2 - CEP2 pointer
//
// Returns:
// -1:	cep1 bind address < cep2 bind address
//	0:	cep1 bind address = cep2 bind address (accounting for wildcards)
//	1:	cep1 bind address > cep2 bind address
int
CepListenAddrCompare(uint64 key1, uint64 key2)
{
	IN CM_CEP_OBJECT* pCEP1 = (CM_CEP_OBJECT*)(uintn)key1;
	IN CM_CEP_OBJECT* pCEP2 = (CM_CEP_OBJECT*)(uintn)key2;
	int res;

	if (pCEP1->SID < pCEP2->SID)
		return -1;
	else if (pCEP1->SID > pCEP2->SID)
		return 1;
	res = WildcardGidCompare(&pCEP1->PrimaryPath.LocalGID, &pCEP2->PrimaryPath.LocalGID);
	if (res != 0)
		return res;
	res = WildcardCompareU64(pCEP1->PrimaryPath.LocalLID, pCEP2->PrimaryPath.LocalLID);
	if (res != 0)
		return res;
	if (pCEP1->bPeer && pCEP2->bPeer)
	{
		res = CompareU64(pCEP1->LocalEndPoint.QPN, pCEP2->LocalEndPoint.QPN);
		if (res != 0)
			return res;
		res = CompareU64(pCEP1->LocalEndPoint.EECN, pCEP2->LocalEndPoint.EECN);
		if (res != 0)
			return res;
		res = CompareU64(pCEP1->LocalEndPoint.CaGUID, pCEP2->LocalEndPoint.CaGUID);
		if (res != 0)
			return res;
	}
	res = WildcardGidCompare(&pCEP1->PrimaryPath.RemoteGID, &pCEP2->PrimaryPath.RemoteGID);
	if (res != 0)
		return res;
	res = WildcardCompareU64(pCEP1->PrimaryPath.RemoteLID, pCEP2->PrimaryPath.RemoteLID);
	if (res != 0)
		return res;
	// a length of 0 matches any private data, so this too is a wildcard compare
	if (pCEP1->DiscriminatorLen == 0 || pCEP2->DiscriminatorLen == 0)
		return 0;
	res = CompareU64(pCEP1->DiscriminatorLen, pCEP2->DiscriminatorLen);
	if (res != 0)
		return res;
	res = MemoryCompare(pCEP1->Discriminator, pCEP2->Discriminator, pCEP1->DiscriminatorLen);
	return res;
}

// A qmap key_compare function to search the ListenMap for a match with
// a given REQ
//
// key1 - CEP pointer
// key2 - REQ pointer
//
// Returns:
// -1:	cep1 bind address < req remote address
//	0:	cep1 bind address = req remote address (accounting for wildcards)
//	1:	cep1 bind address > req remote address
//
// The QPN/EECN/CaGUID are not part of the search, hence multiple Peer Connects
// could be matched (and one which was started earliest should be then linearly
// searched for among the neighbors of the matching CEP)
int
CepReqAddrCompare(uint64 key1, uint64 key2)
{
	IN CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)(uintn)key1;
	IN CMM_REQ* pREQ = (CMM_REQ*)(uintn)key2;
	int res;

	if (pCEP->SID < pREQ->ServiceID)
		return -1;
	else if (pCEP->SID > pREQ->ServiceID)
		return 1;
	// local and remote is from perspective of sender (remote node in this
	// case, so we compare local to remote and visa versa
	res = WildcardGidCompare(&pCEP->PrimaryPath.LocalGID, &pREQ->PrimaryRemoteGID);
	if (res != 0)
		return res;
	res = WildcardCompareU64(pCEP->PrimaryPath.LocalLID, pREQ->PrimaryRemoteLID);
	if (res != 0)
		return res;
	// do not compare QPN/EECN/CaGUID
	res = WildcardGidCompare(&pCEP->PrimaryPath.RemoteGID, &pREQ->PrimaryLocalGID);
	if (res != 0)
		return res;
	res = WildcardCompareU64(pCEP->PrimaryPath.RemoteLID, pREQ->PrimaryLocalLID);
	if (res != 0)
		return res;
	// a length of 0 matches any private data, so this too is a wildcard compare
	if (pCEP->DiscriminatorLen == 0)
		return 0;
	res = MemoryCompare(pCEP->Discriminator, pREQ->PrivateData+pCEP->DiscrimPrivateDataOffset, pCEP->DiscriminatorLen);
	return res;
}

// A qmap key_compare function to search the ListenMap for a match with
// a given SIDR_REQ
//
// key1 - CEP pointer
// key2 - SIDR_REQ pointer
//
// Returns:
// -1:	cep1 bind address < cep2 bind address
//	0:	cep1 bind address = cep2 bind address (accounting for wildcards)
//	1:	cep1 bind address > cep2 bind address
//
// The QPN/EECN/CaGUID are not part of the search.
int
CepSidrReqAddrCompare(uint64 key1, uint64 key2)
{
	IN CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)(uintn)key1;
	IN CMM_SIDR_REQ* pSIDR_REQ = (CMM_SIDR_REQ*)(uintn)key2;
	int res;

	if (pCEP->SID < pSIDR_REQ->ServiceID)
		return -1;
	else if (pCEP->SID > pSIDR_REQ->ServiceID)
		return 1;
	// GID and LIDs are wildcarded/not available at this time
	// do not compare QPN/EECN/CaGUID
	// a length of 0 matches any private data, so this too is a wildcard compare
	if (pCEP->DiscriminatorLen == 0)
		return 0;
	res = MemoryCompare(pCEP->Discriminator, pSIDR_REQ->PrivateData+pCEP->DiscrimPrivateDataOffset, pCEP->DiscriminatorLen);
	return res;
}

