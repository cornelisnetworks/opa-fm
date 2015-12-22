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



#include <datatypes.h>
#include <imemory.h>
#include <ilist.h>
#include <ievent.h>
#include <stl_sd.h>
#include <sdi.h>
#include <ibt.h>
#include <ib_generalServices.h>
#include <sdrequest.h>

//
//Define global context for each of MGMT Class that we support
//

SD_SERVICECONTEXT SdContext_SA = {0};
SD_SERVICECONTEXT SdContext_Vend = {0};


IB_HANDLE GsiSAHandle = NULL;

IB_HANDLE sa_poolhandle = NULL;
IB_HANDLE sa_represp_poolhandle = NULL;

static boolean
map_tranid_queryelement( LIST_ITEM *pItem, void *ptrid );

FSTATUS
InitializeIbt(
    void
    )
/*++
Routine Description.

Arguments:

--*/
{
    FSTATUS Fstatus;

    _DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, InitializeIbt);

	if((Fstatus = RegisterWithGSI()) !=FSUCCESS) 
	{
		_DBG_ERROR(("Cannot register with GSI, exiting\n"));
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);		
		return(Fstatus);
	}

    _DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);		
    return(Fstatus);

}

FSTATUS
RegisterWithGSI(
	void
	)
/*++
--*/
{
	FSTATUS Status;
	uint32 BufSize;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, RegisterWithGSI);

	//Initialize the Subnet Data Interface context for SA
	SdContext_SA.MgmtClass = MCLASS_SUBN_ADM;


	Status = iba_gsi_register_class( 
			MCLASS_SUBN_ADM,				// MgmtClass
			IB_SUBN_ADM_CLASS_VERSION,		// Mgmtclassversion of GSI
			GSI_REGISTER_REPORT_PROCESSOR,	// Register as a report processor
			TRUE, 							// SAR
			(void *)&SdContext_SA,
			SdGsaSendCompleteCallback,
			SdGsaRecvCallback,
			&GsiSAHandle
			);


	if(Status != FSUCCESS)
	{
		_DBG_ERROR(("Register with GSI failed for SubnetAdmin Class, exiting\n"));
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);		
		return(Status);
	}

	BufSize = sizeof(SA_MAD);

	_DBG_INFO(("The size of SA_MAD is <0x%x>\n", (int)sizeof(SA_MAD)));

	Status = iba_gsi_create_dgrm_pool(
                    GsiSAHandle,
                    SD_MAX_SA_NDGRM, //64 elements
                    1, // one buffer per element
                    &BufSize,
                    sizeof(void *), //Context size.
                    &sa_poolhandle
                    );


	if(Status != FSUCCESS)
	{
		_DBG_ERROR(("Create Dgrm Pool failed for SA, exiting\n"));
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);		
		return(Status);
	}

	Status = iba_gsi_create_dgrm_pool(
					GsiSAHandle,
					SD_MAX_RESP_NDGRM, //64 elements
					1, // one buffer per element
					&BufSize,
					sizeof(void *), //Context size.
					&sa_represp_poolhandle
					);


	if(Status != FSUCCESS)
	{
		_DBG_ERROR(("Create Dgrm Pool failed for SA Report Resp, exiting\n"));
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
		return(Status);
	}

	_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);		
	return(Status);
}

void 
DeRegisterWithGSI(
	void
	)
/*++
Routine Description.

Arguments:

--*/
{
	if(sa_represp_poolhandle)
	{
		iba_gsi_destroy_dgrm_pool(sa_represp_poolhandle);
		sa_represp_poolhandle = NULL;
	}

	if(sa_poolhandle) 
	{
		iba_gsi_destroy_dgrm_pool(sa_poolhandle);
		sa_poolhandle = NULL;
	}
	
	
	if(GsiSAHandle) 
	{
		iba_gsi_deregister_class(GsiSAHandle);
		GsiSAHandle = NULL;
	}
}

/*++
Routine Description.

This is invoked when GSA gets a packet sent by this GSI client
(since we are not a manager, we get no unsolicited responses).
The DgrmList will contain a single message.
This could be a single packet message or an RMPP message.
For an RMPP message each RMPP DATA packet is a separate Element in DgrmList.
Each Element has its own GRH header, MAD header, RMPP Header SA_HDR
and the actual SA payload.
this GsiCoalesceDgrm route will help us simply this structure.

The dgrmlist given to us comes from the GSI receive pool.  As such each element
is organized as follows:
	Element
		Buffer - GRH
		Buffer - MAD_BLOCK_SIZE MAD packet

Arguments:
	ServiceContext - handle for our Gsi registration
	DgrmList - the message being sent to us

--*/
FSTATUS
SdGsaRecvCallback(
	void					*ServiceContext,
	IBT_DGRM_ELEMENT		*pDgrmList
	)
{
	PSD_SERVICECONTEXT SdContext = (PSD_SERVICECONTEXT)ServiceContext;
	uint32 messageSize = 0;
	uint8 * buffer = NULL;
	PQueryDetails pQueryElement;
	MAD_RMPP *pMadSar;
	IB_CLASS_PORT_INFO *pClassPortInfo;
	LIST_ITEM *pListItem;
	uint32 classHeaderSize = 0;	// per packet class specific header
	uint8 classVersion = 0;
	uint32 madBlockSize;
	uint32 rmppGsDataSize;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SdGsaRecvCallback);

	ASSERT(pDgrmList);

	// check the service context MgmtClass value
	if ( SdContext->MgmtClass == MCLASS_SUBN_ADM)
	{
		_DBG_INFO(("Receiving a SubnetAdm datagram element\n"));
		classHeaderSize = sizeof(SA_HDR);
		classVersion = IB_SUBN_ADM_CLASS_VERSION;
	} else {
		_DBG_INFO(("Receiving datagram for wrong class <%d>! <Discard>\n",
				SdContext->MgmtClass));
		_DBG_BREAK;
		goto badmad;
	}

	pMadSar = (MAD_RMPP *)GsiDgrmGetRecvMad(pDgrmList);
	if (!pMadSar)
	{
		 _DBG_WARNING(("<dgrm 0x%p> Empty MAD passed in!\n", _DBG_PTR(pDgrmList)));
		goto badmad;
	}

	// Sanity check base mad header
	{
		uint8 BaseVer=0;
		uint8 MgmtClass=0;
		uint8 ClassVer=0;

		MAD_GET_VERSION_INFO((MAD*)pMadSar, &BaseVer, &MgmtClass, &ClassVer);
		if ( ((BaseVer != IB_BASE_VERSION) && (BaseVer != STL_BASE_VERSION)) ||
			 (MgmtClass != SdContext->MgmtClass) ||
			 (ClassVer != classVersion))
		{
			_DBG_WARNING(("<mad 0x%p> Invalid base version, mgmt class or class version <bv %d mc %d cv %d>\n", 
						_DBG_PTR(pMadSar), _DBG_PTR(BaseVer), _DBG_PTR(MgmtClass), ClassVer));
			goto badmad;
		}
	}
	
	if (MAD_IS_REQUEST((MAD*)pMadSar))
	{
		ProcessRequest(ServiceContext, pDgrmList);
		// FCOMPLETED will cause GSI to return the Dgrm to its recv pool
		return FCOMPLETED;
	}

	// Lookup the Query Element using the transaction ID
	SpinLockAcquire(&pQueryList->m_Lock);
	{
		uint64 TempTranID;

		TempTranID = (uint64)(pMadSar->common.TransactionID >> 8);
		_DBG_INFO(("The TransactionID obtained is <%"PRIx64">\n",TempTranID));
		if ((pListItem = QListFindItem(
			&pQueryList->m_List,map_tranid_queryelement, &TempTranID)) == NULL) 
		{
			// if it is not in the list assume that we have already 
			// completed this pQueryElement
			_DBG_WARNING(("Received Message with no outstanding query! <Discard>\n"));
			goto done;
		}
	}
	pQueryElement = (PQueryDetails)QListObj(pListItem);
	_DBG_INFO(("Response Turnaround Time: %"PRId64" usec\n",
					GetTimeStamp() - pQueryElement->CmdSentTime));
	if( pQueryElement->QState != WaitingForResult)
	{
		_DBG_INFO(("pQueryElement is not in WaitingForResult State\n"));
		goto done;
	}

	if(pMadSar->common.u.NS.Status.AsReg16 == MAD_STATUS_BUSY
		&& pQueryElement->RetriesLeft)
	{
		_DBG_INFO(("pQueryElement going into BusyRetryDelay State\n"));
		// leave on queue, timer will fire and trigger retry
		// don't decrement OutStanding, hence while busy we won't attempt to
		// start any other queries either
		pQueryElement->QState = BusyRetryDelay; // causes use of a new timeout
		goto done;
	}

	pQueryElement->QState = ProcessingResponse;	// holds our reference
	// unlock so we can prempt during coallese
	SpinLockRelease(&pQueryList->m_Lock);

	rmppGsDataSize = (pMadSar->common.BaseVersion == IB_BASE_VERSION)
		? IBA_RMPP_GS_DATASIZE : STL_RMPP_GS_DATASIZE;
	madBlockSize =  (pMadSar->common.BaseVersion == IB_BASE_VERSION)
		? IB_MAD_BLOCK_SIZE : STL_MAD_BLOCK_SIZE;

	if(pDgrmList->TotalRecvSize > (sizeof(IB_GRH) + madBlockSize))
	{
		// RMPP response, coalesce the pieces into a single buffer
		messageSize = iba_gsi_coalesce_dgrm2(pDgrmList, &buffer,classHeaderSize,
						IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION);
		if (messageSize == 0)
		{
			_DBG_ERROR(("Invalid RMPP response received! <Discard>\n"));
		} else if (buffer == NULL) {
			_DBG_ERROR(("Can't allocate %d byte RMPP response buffer! <Discard>\n",messageSize));
			messageSize = 0;	// flags failure
		}
		pMadSar = (MAD_RMPP *)(buffer + sizeof(IB_GRH));
	} else {
		messageSize = sizeof(IB_GRH) 
	   				+ sizeof(MAD_COMMON) 
    					+ sizeof(RMPP_HEADER)
    					+ pMadSar->RmppHdr.u2.PayloadLen;	// includes SA_HDRs
		if (pMadSar->RmppHdr.u2.PayloadLen > rmppGsDataSize)
		{
			_DBG_ERROR(("Invalid PayloadLen for single packet message: %u! <Discard>\n", pMadSar->RmppHdr.u2.PayloadLen));
			messageSize = 0;	// flags failure

		}
	}
	SpinLockAcquire(&pQueryList->m_Lock);
	if (FreeCancelledQueryElement(pQueryElement))
		goto done;
	// restore state so failures will cause retry/timeout
	pQueryElement->QState = WaitingForResult;
	if (messageSize == 0)
	{
		// allocate failure or invalid response
		goto done;
	}
	if( pQueryElement->IsSelfCommand) 
	{
		// Internal commands, not requested by an SubnetData user
		_DBG_INFO(("This is a self issued command\n"));
		if(pMadSar->common.u.NS.Status.AsReg16 != MAD_STATUS_SUCCESS)
		{
			_DBG_ERROR(("Mad status = <0x%x: %s>, Port 0x%016"PRIx64"\n",
						pMadSar->common.u.NS.Status.AsReg16,
						_DBG_PTR(iba_mad_status_msg(pMadSar->common.u.NS.Status)),
						pQueryElement->PortGuid));
			// leave on queue, timer will timeout and retry
			goto done;
		}
		
		////////////////
		switch (pQueryElement->SdSentCmd) 
		{
		case GetClassportInfo:
			{
			CA_PORT *pCaPort;
			
			pClassPortInfo = (IB_CLASS_PORT_INFO *)((SA_MAD*)pMadSar)->Data;
			_DBG_DUMP_MEMORY(_DBG_LVL_INFO, ("Class Port Info Response"),
								pClassPortInfo, sizeof(IB_CLASS_PORT_INFO));
			BSWAP_IB_CLASS_PORT_INFO(pClassPortInfo);
			
			// Update Ca Port with the details of the redirected info
			SpinLockAcquire(pCaPortListLock);
			pCaPort = GetCaPort(pDgrmList->PortGuid);

			// spec was not real clear if a non-redirect should supply 0
			// or should provide the default location.
			// So if LID is 0, we use the default location
			// BUGBUG handle redirectGID/GRH case
			if (pClassPortInfo->RedirectLID)
			{
				pCaPort->RedirectedQP = pClassPortInfo->u3.s.RedirectQP;
				pCaPort->RedirectedQKey = pClassPortInfo->Redirect_Q_Key;
				pCaPort->RedirectedLid = pClassPortInfo->RedirectLID;
				pCaPort->RedirectedSL = (uint8)pClassPortInfo->u2.s.RedirectSL;
				// IB does not have Static rate in ClassPortInfo
				// so we can't be sure what rate of remote port is
				// we use a constant value for GSI QPs
				pCaPort->RedirectedStaticRate = IB_STATIC_RATE_GSI;
				// BUGBUG - get PKey Index from Port Info
				pCaPort->RedirectedPkeyIndex = 0; // TBD lookup in Pkey Table for port
												  // pClassPortInfo->Redirect_P_Key;
			} else {
				pCaPort->RedirectedQP = pCaPort->RemoteQp;
				pCaPort->RedirectedQKey = pCaPort->RemoteQKey;
				pCaPort->RedirectedLid = pCaPort->Lid;
				pCaPort->RedirectedSL = pCaPort->ServiceLevel;
				pCaPort->RedirectedStaticRate = pCaPort->StaticRate;
				pCaPort->RedirectedPkeyIndex = pCaPort->PkeyIndex;
			}

			AtomicDecrementVoid(&Outstanding);
			_DBG_INFO(("QueryElement %p Completed: %u Outstanding\n",
				_DBG_PTR(pQueryElement),
				AtomicRead(&Outstanding)));
			QListRemoveItem( &pQueryList->m_List, pListItem );
			//For SA after ClassPortInfo is done, we are ready
			_DBG_INFO(("InfoProviders for SA is initialized to TRUE\n"));
			pCaPort->bInitialized = TRUE;
			
			SpinLockRelease(pCaPortListLock);
			SpinLockRelease(&pQueryList->m_Lock);
			FreeQueryElement(pQueryElement);
			goto unlocked;
			break;
			}
		default:
			DEBUG_ASSERT(0);
			_DBG_ERROR(("Unknown self command! <Discard>\n"));
			break;
		} //End Switch
	} else {
		// SubnetData user issued command, callback the receive handler
		AtomicDecrementVoid(&Outstanding);
		_DBG_INFO(("QueryElement %p Completed: %u Outstanding\n",
			_DBG_PTR(pQueryElement),
			AtomicRead(&Outstanding)));

		// pQueryList->m_lock held, pQueryElement still on list
		// Note RecvHandler may choose to release and reacquire lock for
		// example to do premptable allocates or frees.
		// lock is held during call so RecvHandler can be certain pQueryElement
		// will not be deleted or state changed unless RecvHandler chooses to
		// release lock
		// QueryElement is in WaitingForResult state
		SubnetAdmRecv(pQueryElement, pMadSar, messageSize - sizeof(IB_GRH), pDgrmList);
		// RecvHandler could have freed pQueryElement, so can't use it anymore
	}
	
done:
	// in some situations when we get here the pQueryElement has been freed
	SpinLockRelease(&pQueryList->m_Lock);
unlocked:
	if(buffer != NULL)
	{
		MemoryDeallocate(buffer);
		buffer = NULL;
	}
	SdSendNextOutstanding();	//pQueryElement may be free, can't deref
badmad:
	// FCOMPLETED will cause GSI to return the Dgrm to its recv pool
	return FCOMPLETED;
}

// helper for list search, compare transaction id in list item to one
// provided. Since arg is void*, we must use pointers to uint64 TIDs
static boolean
map_tranid_queryelement( LIST_ITEM *pItem, void *ptrid )
{
	PQueryDetails piQuery = (PQueryDetails)pItem->pObject;
	uint64 *pTempTranId = (uint64*)ptrid;

	uint64 SrcTranId = piQuery->u.pMad->common.TransactionID >> 8;
	uint64 DestTranId = (*pTempTranId);
	_DBG_INFO(("SrcTranID =<%"PRIx64"> DestTranID=<%"PRIx64">\n",\
		SrcTranId,DestTranId));

	return (SrcTranId == DestTranId);
}


/*++
Routine Description.

Arguments:

--*/
void
SdGsaSendCompleteCallback(
	void					*ServiceContext,
	IBT_DGRM_ELEMENT		*pDgrmList
	)
{
	PSD_SERVICECONTEXT SdContext = (PSD_SERVICECONTEXT)ServiceContext;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SdGsaSendCompleteCallback);

	ASSERT(pDgrmList);
	ASSERT( SdContext->MgmtClass == MCLASS_SUBN_ADM);

#ifdef  SA_ALLOC_DEBUG
	if (iba_gsi_dgrm_pool_count(sa_poolhandle) < 50) MsgOut("Dgrm send callback\n");
#endif
	if(SdContext->MgmtClass == MCLASS_SUBN_ADM)
	{
		_DBG_INFO(("GsiSendCompleteCallback called for SUBNETADM DGRM Element\n"));
	}

	iba_gsi_dgrm_pool_put( pDgrmList); 
	
	_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);		
	return;
}
