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

// Private header file
#include "bmamain.h"
#include "bmadebug.h"
#include "bma_provider.h"

static void BmaSetClassPortInfo(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo);
static void BmaGetClassPortInfo(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo);
static void BmaSetBKeyInfo(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo);
static void BmaGetBKeyInfo(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo);
static void BmaWriteVpd(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo);
static void BmaReadVpd(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo);
static void BmaGetModuleStatus(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo);
static void BmaOem(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo);

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
BmaSendCallback(
	void				*ServiceContext,
	IBT_DGRM_ELEMENT	*pDgrmList
	)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, BmaSendCompleteCallback);

	// Return the element list to the free pool.
	iba_gsi_dgrm_pool_put(pDgrmList);

	//DbgMessage(VERBOSE, ("BmaSendCallback: Returning.\n"));
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

    return;

} // BmaSendCompleteCallback()


//////////////////////////////////////////////////////////////////////////
// BmaRecvCallback
//
// GSA calls this routine when a Bma datagram is received.
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
BmaRecvCallback(
	void				*ServiceContext,
	IBT_DGRM_ELEMENT	*pDgrmList
	)
{
	FSTATUS status;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, BmaRecvCallback);
	
	ASSERT(pDgrmList);

	// Sanity checks
	if (ServiceContext != g_BmaGlobalInfo)
	{
		_DBG_ERROR(("<context %p> Invalid CM Context passed in!!!\n", _DBG_PTR(ServiceContext)));

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		status = FINVALID_PARAMETER;
	}
	else if (! CmdThreadQueue(&g_BmaGlobalInfo->RecvThread, pDgrmList))
		status = FINSUFFICIENT_RESOURCES;
	else
    	status = FPENDING;

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return status;
}

void BmaFreeCallback(IN void* Context, IN void* Cmd)
{
	IBT_DGRM_ELEMENT	*pDgrmList = (IBT_DGRM_ELEMENT*)Cmd;
	ASSERT(Context == g_BmaGlobalInfo);

	DgrmPoolPut(pDgrmList);
}

void BmaThreadCallback(IN void* Context, IN void* Cmd)
{
	IBT_DGRM_ELEMENT	*pDgrmList = (IBT_DGRM_ELEMENT*)Cmd;
	IBT_DGRM_ELEMENT	*pRecvDgrm;
	IBT_DGRM_ELEMENT	*pSendDgrm;
	BM_MAD				*pMad=NULL;
	BM_MAD				*pBmGmp;
	FSTATUS				fstatus;
	IB_HANDLE			qp1Handle;
	uint8				portNumber;
	uint32				Count;
	boolean				BKeyMatch;
	BMA_DEV_INFO		*pDevInfo;
	BMA_PORT_INFO		*pPortInfo;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, BmaThreadCallback);

	ASSERT(Context == g_BmaGlobalInfo);

	//
	// iterate thru each datagrams/mads in the list
	//
	for (pRecvDgrm = pDgrmList;
		 pRecvDgrm != NULL;
		 pRecvDgrm = (IBT_DGRM_ELEMENT *)pRecvDgrm->Element.pNextElement)
	{

		// Verify the size of this datagram.
		if (pRecvDgrm->TotalRecvSize != MAD_BLOCK_SIZE + sizeof(IB_GRH))
		{
			_DBG_ERROR(("<dgrm %p> Datagram passed less than 256 bytes <len %d>\n",
							_DBG_PTR(pRecvDgrm), pRecvDgrm->TotalRecvSize));
			// Drop this datagram.
			continue;
		}

		ASSERT(pRecvDgrm->Element.pBufferList);
		ASSERT(GsiDgrmGetRecvGrh(pRecvDgrm));
		pMad = (BM_MAD*)GsiDgrmGetRecvMad(pRecvDgrm);
		if (!pMad)
		{
			_DBG_ERROR(("<dgrm %p> Empty MAD passed in!\n", _DBG_PTR(pRecvDgrm)));
			continue;
		}

		// Sanity checks.
		if ( (pMad->common.BaseVersion  != IB_BASE_VERSION)	|| 
			 (pMad->common.MgmtClass	!= MCLASS_BM)	|| 
			 (pMad->common.ClassVersion != IB_BM_CLASS_VERSION) )
		{
			_DBG_ERROR(("<mad %p> Invalid base version, mgmt class or class version <bv %d mc %d cv %d>\n", 
							_DBG_PTR(pMad),
							pMad->common.BaseVersion,
							pMad->common.MgmtClass,
							pMad->common.ClassVersion));
			// Drop this datagram.
			continue;
		}

		if ( FSUCCESS != GetGsiContextFromPortGuid(pRecvDgrm->PortGuid, &qp1Handle, &portNumber) )
		{
			// Drop this datagram.
			continue;
		}

		SpinLockAcquire( &g_BmaGlobalInfo->DevListLock );
		// below this point use "goto nextpacket" to drop the packet
		pDevInfo = g_BmaGlobalInfo->DevList;
		pPortInfo = NULL;
		// find the port info, this list should be really short, so this isn't as bad as it looks
		while (pDevInfo && !pPortInfo)
		{
			unsigned i;
			for (i = 0; i < pDevInfo->NumPorts; i++)
			{
				if ( pDevInfo->Port[i].Guid == pRecvDgrm->PortGuid )
				{
					pPortInfo = &pDevInfo->Port[i];
					break;
				}
			}
			if (!pPortInfo)
				pDevInfo = pDevInfo->Next;
		}
		if (!pPortInfo)
		{
			// bad port, drop this datagram
			goto nextpacket;
		}

		BSWAP_BM_HDR( &pMad->BmHdr );

		switch ( pMad->common.mr.AsReg8 )
		{
		case BASEMGT_GET:
		case BASEMGT_SET:
			switch (pMad->common.AttributeID)
			{
			// case BM_ATTRIB_ID_NOTICE:	// only if notice queue supported
			case BM_ATTRIB_ID_CLASS_PORTINFO:
			case BM_ATTRIB_ID_BKEY_INFO:
				// ok
				break;
			default:
				// Drop this datagram (or return an error).
				_DBG_ERROR(("<mad %p> Invalid attribute for method %x\n", pMad->common.mr.AsReg8));
				goto nextpacket;
			}
			break;

		case BASEMGT_TRAP_REPRESS:
			// TODO: No reply, handle right here? (do we need to check the BKey?)
			if (pMad->common.AttributeID != BM_ATTRIB_ID_NOTICE) {
				goto nextpacket;
			}
			break;

		case BASEMGT_SEND:
			switch (pMad->common.AttributeID)
			{
			case BM_ATTRIB_ID_WRITE_VPD:
			case BM_ATTRIB_ID_READ_VPD:
			case BM_ATTRIB_ID_GET_MODULE_STATUS:
			case BM_ATTRIB_ID_OEM:
				// ok
				break;
				// not currently supported, but could be
			case BM_ATTRIB_ID_RESET_IBML:
			case BM_ATTRIB_ID_SET_MODULE_PM_CONTROL:
			case BM_ATTRIB_ID_GET_MODULE_PM_CONTROL:
			case BM_ATTRIB_ID_SET_UNIT_PM_CONTROL:
			case BM_ATTRIB_ID_GET_UNIT_PM_CONTROL:
			case BM_ATTRIB_ID_SET_IOC_PM_CONTROL:
			case BM_ATTRIB_ID_GET_IOC_PM_CONTROL:
			case BM_ATTRIB_ID_SET_MODULE_STATE:
			case BM_ATTRIB_ID_SET_MODULE_ATTENTION:
			case BM_ATTRIB_ID_IB2IBML:
			case BM_ATTRIB_ID_IB2CME:
			case BM_ATTRIB_ID_IB2MME:
			default:
				// Drop this datagram (or return an error).
				_DBG_ERROR(("<mad %p> Invalid attribute for method %x\n", pMad->common.mr.AsReg8));
				goto nextpacket;
			}
			break;

			// we should never get these on incoming packets
		case BASEMGT_REPORT:
		case BASEMGT_REPORT_RESP:
		case BASEMGT_GET_RESP:
		case BASEMGT_TRAP:
		default:
			// Drop this datagram.
			_DBG_ERROR(("<mad %p> Invalid method %x\n", pMad->common.mr.AsReg8));
			goto nextpacket;
		}

		//
		// looks good, check the key
		//
		BKeyMatch = ( pPortInfo->BKey == 0 ) || ( pMad->BmHdr.BKey == pPortInfo->BKey ) ||
					(( pMad->common.mr.AsReg8 == BASEMGT_GET ) && ( !pPortInfo->BKeyProtect ));

		if (!BKeyMatch && ( pMad->common.mr.AsReg8 != BASEMGT_GET ))
		{
			if (pPortInfo->BKeyViolations < 0xffff)
				pPortInfo->BKeyViolations++;
			// TODO: Send BMA_TRAP_BAD_BKEY, and schedule to be repeated
			if (pPortInfo->BKeyLease) {
				// TODO: Start countdown timer for BKeyLease seconds
			}
			// Drop this datagram
			goto nextpacket;
		}

		//
		// Process the valid mad
		//
		if (g_BmaGlobalInfo->DgrmPoolHandle != NULL)
		{
			// Obtain a send datagram element.
			Count = 1;
			fstatus = iba_gsi_dgrm_pool_get(
							g_BmaGlobalInfo->DgrmPoolHandle,
							&Count,
							&pSendDgrm);
			if (fstatus != FSUCCESS)
			{
				_DBG_ERROR(("BmaRecvCallback: GsiDgrmPoolGet unsuccessful.\n"));
				// Drop this datagram.
				goto nextpacket;
			}
		}
		else
		{
			_DBG_ERROR(("BmaRecvCallback: DgrmPoolHandle == NULL.\n"));
			// Drop this datagram.
			goto nextpacket;
		}

		// Set endpoint information.
		GsiDgrmAddrCopy(pSendDgrm, pRecvDgrm);

		// Set the response datagram size.
		pSendDgrm->Element.pBufferList->ByteCount = MAD_BLOCK_SIZE;

		// Initalize a pointer to the send perf management packet.
		pBmGmp = (BM_MAD*)GsiDgrmGetSendMad(pSendDgrm);

		// Initialize the send GMP.
		MemoryCopy((char*)pBmGmp, (char*)pMad, sizeof(BM_MAD));
		pBmGmp->common.u.NS.Status.AsReg16 = MAD_STATUS_SUCCESS;

		if (fstatus == FSUCCESS)
		{
			// swap BMData to host byte order
			switch (pBmGmp->common.AttributeID)
			{
			case BM_ATTRIB_ID_CLASS_PORTINFO:
				if ( pMad->common.mr.AsReg8 == BASEMGT_SET ) {
					BSWAP_IB_CLASS_PORT_INFO((IB_CLASS_PORT_INFO*)&pBmGmp->BMData);
					BmaSetClassPortInfo(pBmGmp, pPortInfo);
				}
				else {
					BmaGetClassPortInfo(pBmGmp, pPortInfo);
				}
				BSWAP_IB_CLASS_PORT_INFO((IB_CLASS_PORT_INFO*)&pBmGmp->BMData);
				break;

			case BM_ATTRIB_ID_BKEY_INFO:
				if ( pMad->common.mr.AsReg8 == BASEMGT_SET ) {
					BSWAP_BKEY_INFO((BKEY_INFO*)&pBmGmp->BMData);
					BmaSetBKeyInfo(pBmGmp, pPortInfo);
				}
				else {
					BmaGetBKeyInfo(pBmGmp, pPortInfo);
				}
				if (!BKeyMatch) {
					((BKEY_INFO*)&pBmGmp->BMData)->B_Key = 0;
				}
				BSWAP_BKEY_INFO((BKEY_INFO*)&pBmGmp->BMData);
				break;

			case BM_ATTRIB_ID_WRITE_VPD:
				if (pMad->common.AttributeModifier == BM_ATTRIB_MOD_REQUEST) {
					BSWAP_BM_SEND((BM_SEND*)&pBmGmp->BMData);
					BmaWriteVpd(pBmGmp, pPortInfo);
					BSWAP_BM_SEND((BM_SEND*)&pBmGmp->BMData);
				}
				else {
					pBmGmp->common.u.NS.Status.AsReg16 = MAD_STATUS_INVALID_ATTRIB;
				}
                // Set the AttributModifier bit to reflect a BMSEND response.
                pBmGmp->common.AttributeModifier = BM_ATTRIB_MOD_RESPONSE;
				break;

			case BM_ATTRIB_ID_READ_VPD:
				if (pMad->common.AttributeModifier == BM_ATTRIB_MOD_REQUEST) {
					BSWAP_BM_SEND((BM_SEND*)&pBmGmp->BMData);
					BmaReadVpd(pBmGmp, pPortInfo);
					BSWAP_BM_SEND((BM_SEND*)&pBmGmp->BMData);
				}
				else {
					pBmGmp->common.u.NS.Status.AsReg16 = MAD_STATUS_INVALID_ATTRIB;
				}
                // Set the AttributModifier bit to reflect a BMSEND response.
                pBmGmp->common.AttributeModifier = BM_ATTRIB_MOD_RESPONSE;
				break;

			case BM_ATTRIB_ID_GET_MODULE_STATUS:
				if (pMad->common.AttributeModifier == BM_ATTRIB_MOD_REQUEST) {
					BSWAP_BM_SEND((BM_SEND*)&pBmGmp->BMData);
					BmaGetModuleStatus(pBmGmp, pPortInfo);
					BSWAP_BM_SEND((BM_SEND*)&pBmGmp->BMData);
				}
				else {
					pBmGmp->common.u.NS.Status.AsReg16 = MAD_STATUS_INVALID_ATTRIB;
				}
                // Set the AttributModifier bit to reflect a BMSEND response.
                pBmGmp->common.AttributeModifier = BM_ATTRIB_MOD_RESPONSE;
				break;

            case BM_ATTRIB_ID_OEM:
				if (pMad->common.AttributeModifier == BM_ATTRIB_MOD_REQUEST) {
    				BSWAP_BM_SEND((BM_SEND*)&pBmGmp->BMData);
    				BmaOem(pBmGmp, pPortInfo);
    				BSWAP_BM_SEND((BM_SEND*)&pBmGmp->BMData);
                }
                // Set the AttributModifier bit to reflect a BMSEND response.
                pBmGmp->common.AttributeModifier = BM_ATTRIB_MOD_RESPONSE;
				break;

				// not yet supported:
			case BM_ATTRIB_ID_NOTICE:
				// Invalid attribute
				pBmGmp->common.u.NS.Status.AsReg16 = MAD_STATUS_UNSUPPORTED_METHOD_ATTRIB;
                break;

			case BM_ATTRIB_ID_RESET_IBML:
			case BM_ATTRIB_ID_SET_MODULE_PM_CONTROL:
			case BM_ATTRIB_ID_GET_MODULE_PM_CONTROL:
			case BM_ATTRIB_ID_SET_UNIT_PM_CONTROL:
			case BM_ATTRIB_ID_GET_UNIT_PM_CONTROL:
			case BM_ATTRIB_ID_SET_IOC_PM_CONTROL:
			case BM_ATTRIB_ID_GET_IOC_PM_CONTROL:
			case BM_ATTRIB_ID_SET_MODULE_STATE:
			case BM_ATTRIB_ID_SET_MODULE_ATTENTION:
			case BM_ATTRIB_ID_IB2IBML:
			case BM_ATTRIB_ID_IB2CME:
			case BM_ATTRIB_ID_IB2MME:
			default:
				// Invalid attribute
				pBmGmp->common.u.NS.Status.AsReg16 = MAD_STATUS_UNSUPPORTED_METHOD_ATTRIB;
                // Set the AttributModifier bit to reflect a BMSEND response.
                pBmGmp->common.AttributeModifier = BM_ATTRIB_MOD_RESPONSE;
				break;
			}
		}

		BSWAP_BM_HDR( &pBmGmp->BmHdr );

		// Send the response datagram.
		fstatus = iba_gsi_post_send(g_BmaGlobalInfo->GsaHandle, pSendDgrm);
		if (fstatus != FSUCCESS)
		{
			iba_gsi_dgrm_pool_put(pSendDgrm);
		}

nextpacket:
		SpinLockRelease( &g_BmaGlobalInfo->DevListLock );
	} // for

	DgrmPoolPut(pDgrmList);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
} // BmaRecvCallback()


static void
BmaSetClassPortInfo(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo)
{
	IB_CLASS_PORT_INFO* pClassPortInfo = (IB_CLASS_PORT_INFO*)&pMad->BMData;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, BmaSetClassPortInfo);

	// validate fields
	if (pClassPortInfo->TrapLID && (!pClassPortInfo->u5.s.TrapQP || !(pClassPortInfo->Trap_Q_Key & 0x80000000)))
	{
		pMad->common.mr.AsReg8 = MMTHD_GET_RESP;
		pMad->common.u.NS.Status.AsReg16 = MAD_STATUS_INVALID_ATTRIB;
	}
	else {
		// apply, no locking necessary, Bma thread is only one to use
		pPortInfo->Trap.GID			= pClassPortInfo->TrapGID;
		pPortInfo->Trap.TClass		= pClassPortInfo->u4.s.TrapTClass;
		pPortInfo->Trap.SL			= pClassPortInfo->u4.s.TrapSL;
		pPortInfo->Trap.FlowLabel	= pClassPortInfo->u4.s.TrapFlowLabel;
		pPortInfo->Trap.LID			= pClassPortInfo->TrapLID;
		pPortInfo->Trap.P_Key		= pClassPortInfo->Trap_P_Key;
		pPortInfo->Trap.HopLimit		= pClassPortInfo->u5.s.TrapHopLimit;
		pPortInfo->Trap.QP			= pClassPortInfo->u5.s.TrapQP;
		pPortInfo->Trap.Q_Key		= pClassPortInfo->Trap_Q_Key;

		// generate response
		BmaGetClassPortInfo(pMad, pPortInfo);
	}

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}


static void
BmaGetClassPortInfo(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo)
{
	IB_CLASS_PORT_INFO* pClassPortInfo = (IB_CLASS_PORT_INFO*)&pMad->BMData;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, BmaGetClassPortInfo);

	MemoryClear(pClassPortInfo, sizeof(*pClassPortInfo));
	pClassPortInfo->BaseVersion = IB_BASE_VERSION;
	pClassPortInfo->ClassVersion = IB_BM_CLASS_VERSION;
	pClassPortInfo->CapMask = CLASS_PORT_CAPMASK_TRAP;		// TODO: Is this all?
	pClassPortInfo->u1.s.RespTimeValue = PORT_RESP_TIME_VALUE_MAX; // 8

	// setup trap info
	pClassPortInfo->TrapGID 			= pPortInfo->Trap.GID;
	pClassPortInfo->u4.s.TrapTClass		= pPortInfo->Trap.TClass;
	pClassPortInfo->u4.s.TrapSL			= pPortInfo->Trap.SL;
	pClassPortInfo->u4.s.TrapFlowLabel	= pPortInfo->Trap.FlowLabel;
	pClassPortInfo->TrapLID 			= pPortInfo->Trap.LID;
	pClassPortInfo->Trap_P_Key 			= pPortInfo->Trap.P_Key;
	pClassPortInfo->u5.s.TrapHopLimit  	= pPortInfo->Trap.HopLimit;
	pClassPortInfo->u5.s.TrapQP		 	= pPortInfo->Trap.QP;
	pClassPortInfo->Trap_Q_Key 			= pPortInfo->Trap.Q_Key;

	pMad->common.mr.AsReg8 = MMTHD_GET_RESP;

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}

static void
BmaSetBKeyInfo(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo)
{
	BKEY_INFO* pBKeyInfo = (BKEY_INFO*)&pMad->BMData;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, BmaSetBKeyInfo);

	// Do set
	pPortInfo->BKey 		= pBKeyInfo->B_Key;
	pPortInfo->BKeyProtect	= pBKeyInfo->u1.s.B_KeyProtectBit;
	pPortInfo->BKeyLease	= pBKeyInfo->B_KeyLeasePeriod;

	// Response
	pBKeyInfo->B_KeyViolations = pPortInfo->BKeyViolations;

	pMad->common.mr.AsReg8 = MMTHD_GET_RESP;

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}

static void
BmaGetBKeyInfo(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo)
{
	BKEY_INFO* pBKeyInfo = (BKEY_INFO*)&pMad->BMData;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, BmaGetBKeyInfo);

	MemoryClear(pBKeyInfo, sizeof(*pBKeyInfo));

	pBKeyInfo->B_Key				= pPortInfo->BKey;
	pBKeyInfo->u1.s.B_KeyProtectBit	= pPortInfo->BKeyProtect;
	pBKeyInfo->B_KeyLeasePeriod		= pPortInfo->BKeyLease;
	pBKeyInfo->B_KeyViolations		= pPortInfo->BKeyViolations;

	pMad->common.mr.AsReg8 = MMTHD_GET_RESP;

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}

static void
BmaWriteVpd(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo)
{
	BM_SEND*	pBMSend = (BM_SEND*)&pMad->BMData;
	uint16		seq		= pBMSend->Sequence;
	uint8		dev		= BM_REQ_GET_VPD_DEVICE(pBMSend);
	uint16		bytes	= BM_REQ_GET_VPD_NUM_BYTES(pBMSend);
	uint16		offset	= BM_REQ_GET_VPD_OFFSET(pBMSend);
	uint8		status	= BM_STATUS_OK;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, BmaWriteVpd);

	if ( bytes > BM_WRITE_VPD_MAX_BYTES )
	{
		status = BM_STATUS_ILLEGAL_BYTE_COUNT;
	}
	else
	{
		status = BmaProvider_WriteVPD(dev, bytes, offset, BM_REQ_WRITE_VPD_DATA_PTR(pBMSend));
	}

	// build response packet
	MemoryClear(pBMSend, sizeof(BM_SEND));
	pBMSend->Sequence		= seq;
	pBMSend->SourceDevice	= BM_DEV_MME;
	pBMSend->ParamCount		= 1;
	BM_RSP_SET_STATUS(pBMSend, status);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}

static void
BmaReadVpd(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo)
{
	BM_SEND*	pBMSend = (BM_SEND*)&pMad->BMData;
	uint16		seq		= pBMSend->Sequence;
	uint8		dev		= BM_REQ_GET_VPD_DEVICE(pBMSend);
	uint16		bytes	= BM_REQ_GET_VPD_NUM_BYTES(pBMSend);
	uint16		offset	= BM_REQ_GET_VPD_OFFSET(pBMSend);
	uint8		status	= BM_STATUS_OK;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, BmaReadVpd);

	MemoryClear(pBMSend, sizeof(BM_SEND));

	if ( bytes > BM_READ_VPD_MAX_BYTES )
	{
		status = BM_STATUS_ILLEGAL_BYTE_COUNT;
	}
	else
	{
		status = BmaProvider_ReadVPD(dev, bytes, offset, BM_RSP_READ_VPD_DATA_PTR(pBMSend));
	}

	// build response packet
	pBMSend->Sequence		= seq;
	pBMSend->SourceDevice	= BM_DEV_MME;
	pBMSend->ParamCount		= 1;
	if (status == BM_STATUS_OK)
		pBMSend->ParamCount += bytes;

	BM_RSP_SET_STATUS(pBMSend, status);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}

static void
BmaGetModuleStatus(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo)
{
	BM_SEND*	pBMSend = (BM_SEND*)&pMad->BMData;
	uint16		seq		= pBMSend->Sequence;
	uint8		status	= BM_STATUS_OK;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, BmaGetModuleStatus);

	MemoryClear(pBMSend, sizeof(BM_SEND));

	status = BmaProvider_GetModuleStatus(BM_RSP_MODULE_STATUS_DATA_PTR(pBMSend));

	// build response packet
	pBMSend->Sequence		= seq;
	pBMSend->SourceDevice	= BM_DEV_MME;
	pBMSend->ParamCount		= BM_GET_MODULE_STATUS_BYTES;

	BM_RSP_SET_STATUS(pBMSend, status);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}

static void
BmaOem(BM_MAD* pMad, BMA_PORT_INFO* pPortInfo)
{
	BM_SEND*	pBMSend = (BM_SEND*)&pMad->BMData;
	uint16		seq		= pBMSend->Sequence;
	uint8		status	= BM_STATUS_OK;
	uint32		vendor	= BM_REQ_GET_OEM_VENDOR_ID(pBMSend);
	uint8		numBytes = BM_REQ_GET_OEM_NUM_BYTES(pBMSend);
	uint8		rspBuf[BM_OEM_RSP_MAX_BYTES];

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, BmaOem);

	if (numBytes > BM_OEM_REQ_MAX_BYTES) {
		numBytes = 0;
		status = BM_STATUS_ILLEGAL_BYTE_COUNT;
	}
	else {
		status = BmaProvider_OemCmd(vendor, numBytes, BM_REQ_OEM_DATA_PTR(pBMSend), &numBytes, rspBuf);
	}

	// build response packet
	pBMSend->Sequence		= seq;
	pBMSend->SourceDevice	= BM_DEV_MME;

	BM_RSP_SET_OEM_NUM_BYTES(pBMSend, numBytes);
	BM_RSP_SET_STATUS(pBMSend, status);
	BM_RSP_SET_OEM_VENDOR_ID(pBMSend, vendor);
	MemoryCopy(BM_RSP_OEM_DATA_PTR(pBMSend), rspBuf, min(numBytes, sizeof(pBMSend->Parameters) - 4));

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}

