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


#include "gsamain.h"



//
// GsiPostSendDgrm
//
//
//
// INPUTS:
//
//	ServiceHandle	- Handle returned upon registration
//
//	DgrmList		- List of datagram(s) to be posted for send
//					each must have payload in network byte order
//					however MAD and RMPP_HEADERs should be in host
//					byte order.  On success, GSI owns the buffer and
//					will byte swap back the MAD header prior to send completion
//					on failure the buffer will be restored to host byte order
//					for the MAD and RMPP headers
//
// OUTPUTS:
//
//	None
//
//
//
// RETURNS:
//
//	FSUCCESS
//	FINVALID_PARAMETER
//	FINSUFFICIENT_RESOURCES
//	FINVALID_STATE
//
FSTATUS
iba_gsi_post_send(
	IN	IB_HANDLE				ServiceHandle,
	IN	IBT_DGRM_ELEMENT		*DgrmList
	)
{
	FSTATUS						status=FCANCELED;
	GSA_POST_SEND_LIST			*pPostList;
	GSA_SERVICE_CLASS_INFO		*serviceClass;
	IBT_DGRM_ELEMENT_PRIV		*pDgrmPrivate;

	_DBG_ENTER_LVL (_DBG_LVL_SEND, iba_gsi_post_send);

	serviceClass = (GSA_SERVICE_CLASS_INFO *)ServiceHandle;
	pDgrmPrivate = (IBT_DGRM_ELEMENT_PRIV *)DgrmList;
	
#if defined(DBG) || defined(IB_DEBUG)
	if (GSA_CLASS_SIG != serviceClass->SigInfo )
	{
		_DBG_ERROR (("Bad Class Handle passed by client!!!\n"));
		_DBG_BREAK;
	}

	if ( pDgrmPrivate->MemPoolHandle->ServiceHandle != serviceClass )
	{
		_DBG_ERROR (("Dgrm List is not owned by given Class Handle!!!\n"));
		_DBG_BREAK;
	}
#endif

	// generate a PostSendList from the first datagram in post
	pPostList					= &pDgrmPrivate->PostSendList;

	// Fill in list details
	pPostList->DgrmIn = 0;
	AtomicWrite(&pPostList->DgrmOut, 0);
	pPostList->DgrmList			= DgrmList;
	pPostList->SARRequest		= FALSE;

	// Find out if client is a SAR client
	if ( TRUE == serviceClass->bSARRequired )
	{
		_DBG_PRINT(_DBG_LVL_SEND,( "SAR Client request\n" )); 
		status = DoSARSend( serviceClass, pPostList );
	}

	// We get here if non SAR client or if SAR request was not segmented
	if ( FCANCELED == status )
	{
		IBT_DGRM_ELEMENT			*pDgrmPostList;

		// Get total send requests
		pDgrmPostList			= DgrmList;
		while ( pDgrmPostList )
		{
			pPostList->DgrmIn++;
			pDgrmPostList		= \
				(IBT_DGRM_ELEMENT *)pDgrmPostList->Element.pNextElement;
		}

		status = GsiDoSendDgrm(serviceClass, pPostList);
	}	// SAR

    _DBG_LEAVE_LVL(_DBG_LVL_SEND);
	return status;
}


//
// GsiDoSendDgrm
//
//
//
// INPUTS:
//
//	ServiceClass	- ServiceClass from registration
//						if NULL, assumed to be Dgrms from global RecvQ
//						and 2nd buffer is a single MAD to be posted
//
//	pPostList		- post list info.
//					1st pPostList->DgrmIn dgrms in this list will be posted.
//					each must have payload in network byte order
//					however MAD and RMPP_HEADERs should be in host
//					byte order.  On success, GSI owns the buffer and
//					will byte swap back the MAD header prior to send completion
//					on failure the buffer will be restored to host byte order
//					for the MAD and RMPP headers
//
// OUTPUTS:
//
//	None
//
//
//
// RETURNS:
//
//
FSTATUS
GsiDoSendDgrm(
	GSA_SERVICE_CLASS_INFO		*ServiceClass,
	IN	GSA_POST_SEND_LIST		*pPostList
	)
{
	FSTATUS						status = FINVALID_PARAMETER; // list empty
	IBT_DGRM_ELEMENT_PRIV		*pDgrmPrivate, *pDgrmPrivateNext;
	IB_HANDLE					qp1Handle, cqHandle;
	SPIN_LOCK					*qpLock;
	IB_L_KEY					memLKey;
	MAD							*pMad;
	uint32						dgrmCount, i, dgrmFailed;
	IB_WORK_REQ					workRequest;
	IB_ADDRESS_VECTOR			avInfo;

	_DBG_ENTER_LVL (_DBG_LVL_SEND, GsiDoSendDgrm);

	pDgrmPrivate = (IBT_DGRM_ELEMENT_PRIV *)pPostList->DgrmList;
	
	dgrmCount				= pPostList->DgrmIn;
	dgrmFailed				= 0;			// set a count for bad requests

	while ( dgrmCount-- )
	{
		pDgrmPrivateNext = (IBT_DGRM_ELEMENT_PRIV *)
								pDgrmPrivate->DgrmElement.Element.pNextElement;

		pDgrmPrivate->Base	= pPostList;	// link for completion processing

		ASSERT_VALID_WORKREQID((uintn)pDgrmPrivate);
		workRequest.WorkReqId = BUILD_SQ_WORKREQID((uintn)pDgrmPrivate);
		workRequest.Operation = WROpSend;

		workRequest.DSList = pDgrmPrivate->DsList;

		// zero out the options
  		workRequest.Req.SendUD.Options.AsUINT16 = 0;
		workRequest.Req.SendUD.Options.s.SignaledCompletion = TRUE; 
#if INCLUDE_16B
		workRequest.Req.SendUD.Options.s.SendFMH			= 0;
#endif

		// Fill in user settings
		workRequest.Req.SendUD.QPNumber = pDgrmPrivate->DgrmElement.RemoteQP;
		// TBD check uses, may be safe to enable this line now
		//workRequest.Req.SendUD.Qkey	= pDgrmPrivate->DgrmElement.RemoteQKey;
		workRequest.Req.SendUD.Qkey		= QP1_WELL_KNOWN_Q_KEY;
		workRequest.Req.SendUD.PkeyIndex= pDgrmPrivate->DgrmElement.PkeyIndex;
			
#if defined(DBG) || defined(IB_DEBUG)
		if (ServiceClass && !workRequest.Req.SendUD.QPNumber) 
		{
			AtomicIncrement( &ServiceClass->ErrorMsgs );
			
			if (!(AtomicRead(&ServiceClass->ErrorMsgs) % 20))
			{
				_DBG_ERROR((
					"Invalid QPNumber/Q_Key(substitution provided):\n"
					"\tClass..........:x%x\n"
					"\tSARClient......:%s\n"
					"\tQPNumber.......:x%x\n"
					"\tQ_Key..........:x%x\n",
					ServiceClass->MgmtClass,
					_DBG_PTR(ServiceClass->bSARRequired ? "TRUE":"FALSE"),
					workRequest.Req.SendUD.QPNumber,
					workRequest.Req.SendUD.Qkey
					));
					
				_DBG_ERROR((
					"\tResponder......:%s\n"
					"\tTrapProcessor..:%s\n"
					"\tReportProcessor:%s\n",
					_DBG_PTR(GSI_IS_RESPONDER(ServiceClass->RegistrationFlags)
						? "TRUE":"FALSE"),
					_DBG_PTR(GSI_IS_TRAP_PROCESSOR(ServiceClass->RegistrationFlags)
						? "TRUE":"FALSE"),
					_DBG_PTR(GSI_IS_REPORT_PROCESSOR(ServiceClass->RegistrationFlags)
						? "TRUE":"FALSE")
					));
			}

			workRequest.Req.SendUD.Qkey		= QP1_WELL_KNOWN_Q_KEY;
			workRequest.Req.SendUD.QPNumber = 1;
		}
		ASSERT(workRequest.Req.SendUD.QPNumber);

		if (workRequest.Req.SendUD.QPNumber	!= 1)
		{
			_DBG_WARN((
				"Send to RedirectedQP\n"
				"\tQPNumber.....:x%x\n",
				workRequest.Req.SendUD.QPNumber
				));
		}
#endif		
		// Fill in Address Vector Info
		avInfo.PortGUID			= pDgrmPrivate->DgrmElement.PortGuid;
		avInfo.DestLID			= pDgrmPrivate->DgrmElement.RemoteLID;
		avInfo.PathBits			= pDgrmPrivate->DgrmElement.PathBits;
		avInfo.ServiceLevel		= pDgrmPrivate->DgrmElement.ServiceLevel;
		avInfo.StaticRate		= pDgrmPrivate->DgrmElement.StaticRate;
		avInfo.GlobalRoute		= pDgrmPrivate->DgrmElement.GlobalRoute;

		if ( TRUE == avInfo.GlobalRoute )
		{
			/* structure copy */
			avInfo.GlobalRouteInfo	= pDgrmPrivate->DgrmElement.GRHInfo;
		}

		status = GetCaContextFromAvInfo(
					&avInfo,
					pDgrmPrivate->MemPoolHandle->MemList->CaMemIndex,
					&workRequest.Req.SendUD.AVHandle,
					&qp1Handle,
					&cqHandle,
					&qpLock,
					&memLKey );

		if ( FSUCCESS == status )
		{
			pDgrmPrivate->AvHandle = workRequest.Req.SendUD.AVHandle;

			// fill in scatter/gather Ds list details
			if (! ServiceClass)
			{
				// Rmpp replies and GSI internal replies
				IB_LOCAL_DATASEGMENT		*pDsList = workRequest.DSList;

				ASSERT( pDgrmPrivate->MemPoolHandle->ReceivePool);
				pMad = GsiDgrmGetRecvMad(&pDgrmPrivate->DgrmElement);
				workRequest.DSListDepth = 1;
				pDsList[0].Address				= (uintn)pMad;
				pDsList[0].Lkey					= memLKey;
				pDsList[0].Length				=
					GsiDgrmGetRecvMadByteCount(&pDgrmPrivate->DgrmElement);
				workRequest.MessageLen =
					GsiDgrmGetRecvMadByteCount(&pDgrmPrivate->DgrmElement);
			}
			else if ( TRUE != ServiceClass->bSARRequired )
			{
				// Direct sends from other callers
				IBT_BUFFER					*pBuffer;
				IB_LOCAL_DATASEGMENT		*pDsList = workRequest.DSList;

				pBuffer = pDgrmPrivate->DgrmElement.Element.pBufferList;

				workRequest.DSListDepth = \
							pDgrmPrivate->MemPoolHandle->BuffersPerElement;
				workRequest.MessageLen = 0;
				for (i=0; i<workRequest.DSListDepth; i++ )
				{
					pDsList[i].Address = (uintn)pBuffer->pData;
					pDsList[i].Lkey = memLKey;
					pDsList[i].Length = pBuffer->ByteCount;

					workRequest.MessageLen += pBuffer->ByteCount;
					pBuffer = pBuffer->pNextBuffer;
				}

				// Validate that buffers per element are sizeof MAD for non
				// SAR clients
				if ( workRequest.MessageLen > sizeof(MAD) )
				{
					_DBG_ERROR((
					"Bad Non SAR Client posted buffer > sizeof(MAD)!\n"));
					_DBG_BREAK;
				}
				pMad = GsiDgrmGetSendMad(&pDgrmPrivate->DgrmElement);
			} else {
				// invoked for single packet GSI SAR send
				// SAR will Always be 3: MAD_COMMON, SAR_HDR, DATA
				IBT_BUFFER					*pBuffer;

				pBuffer = pDgrmPrivate->DgrmElement.Element.pBufferList;
				workRequest.DSListDepth = 3;

				// Fill in MAD Common info
				workRequest.DSList[0].Address = (uintn)pBuffer->pData;
				workRequest.DSList[0].Lkey = memLKey;
				workRequest.DSList[0].Length = sizeof(MAD_COMMON);

				// Fill in SAR Header info
				workRequest.DSList[1].Address = \
					(uintn)(pBuffer->pData) + sizeof(MAD_COMMON);
				workRequest.DSList[1].Lkey = memLKey;
				workRequest.DSList[1].Length = sizeof(RMPP_HEADER);

				// Fill in Data info
				workRequest.DSList[2].Address = \
					(uintn)(pBuffer->pData) + \
					sizeof(MAD_COMMON) +
					sizeof(RMPP_HEADER);
				workRequest.DSList[2].Lkey = memLKey;
				workRequest.DSList[2].Length = pBuffer->ByteCount -
					(sizeof(MAD_COMMON) + sizeof(RMPP_HEADER));
				
				workRequest.MessageLen = pBuffer->ByteCount;
				pMad = GsiDgrmGetSendMad(&pDgrmPrivate->DgrmElement);
			}
			
			// Set Transaction ID

			// If this is a request, save the user's transaction id 
			// and use our own.  We will restore the user's transaction id
			// when the request completes.  If this is not a request,
			// we let the client control the transaction id which may have
			// been remotely generated.
			if( ServiceClass && MAD_IS_REQUEST(pMad))
			{
				// We need to save the TID in case we receive the
				// response before being notified that the send completed.
				pDgrmPrivate->SavedSendTid = pMad->common.TransactionID;

				pMad->common.TransactionID = (uint64)\
					(pMad->common.TransactionID & 0xffffffffffffff00ll);
				pMad->common.TransactionID = \
					pMad->common.TransactionID | ServiceClass->ClientId;
			}

			// Make headers Network Byte order
			BSWAP_MAD_HEADER(pMad);
			if( (ServiceClass && ServiceClass->bSARRequired) )
			{
				BSWAP_RMPP_HEADER ( 
					(RMPP_HEADER *)((uintn)workRequest.DSList[1].Address) );
			} else if( pPostList->SARRequest) {
				// only applies to Rmpp Reply and Gsi reply
				ASSERT(workRequest.DSListDepth == 1);
				BSWAP_RMPP_HEADER ( 
					(RMPP_HEADER *)&(((MAD_RMPP*)pMad)->RmppHdr));
			}

#if defined(DBG) || defined(IB_DEBUG)
			if (__DBG_LEVEL__ & _DBG_LVL_PKTDUMP)
			{
				uint32 Length = workRequest.MessageLen;
				IB_LOCAL_DATASEGMENT *Current = &workRequest.DSList[0];
				for (i = 0; i < workRequest.DSListDepth; i++,Current++)
				{
					_DBG_DUMP_MEMORY(_DBG_LVL_PKTDUMP, ("Send Packet"), (void *)(uintn)Current->Address,
									MIN(Current->Length,Length));
					Length -= MIN(Current->Length,Length);;
				}
			}
#endif

			// Submit to verbs
			pDgrmPrivate->DgrmElement.OnSendQ = TRUE;
			SpinLockAcquire( qpLock );
			status = iba_post_send( qp1Handle, &workRequest ); 
			SpinLockRelease( qpLock );

			if ( FSUCCESS != status )
			{
				pDgrmPrivate->DgrmElement.OnSendQ = FALSE;
				// restore headers to host Byte order
				BSWAP_MAD_HEADER(pMad);
				if( (ServiceClass && ServiceClass->bSARRequired) )
				{
					BSWAP_RMPP_HEADER ( 
						(RMPP_HEADER *)((uintn)workRequest.DSList[1].Address) );
				} else if( pPostList->SARRequest) {
					// only applies to Rmpp Reply and Gsi reply
					ASSERT(workRequest.DSListDepth == 1);
					BSWAP_RMPP_HEADER ( 
						(RMPP_HEADER *)&(((MAD_RMPP*)pMad)->RmppHdr));
				}

				// release AV
				(void)PutAV(NULL,pDgrmPrivate->AvHandle);
				pDgrmPrivate->DgrmElement.Element.Status = status;
				AtomicIncrementVoid( &pPostList->DgrmOut );
				dgrmFailed++;
			}
		} else {
			pDgrmPrivate->DgrmElement.Element.Status = status;
			AtomicIncrementVoid( &pPostList->DgrmOut );
			dgrmFailed++;
		}

		pDgrmPrivate = pDgrmPrivateNext;
	}

	// If all requests failed, fail the call and pass it back to user.
	// There is no way of passing all failed entries through the callback
	if ( pPostList->DgrmIn && dgrmFailed == pPostList->DgrmIn )
	{
		status = FINVALID_STATE;
	}

    _DBG_LEAVE_LVL(_DBG_LVL_SEND);
	return status;
}

// uses pDgrm as a template for building an GET_RESP reply
// reporting a Invalid Class Version status
// MAD header and AV information is taken from pDgrm
// pDgrm is unaffected, assumes pDgrm is in Host byte order for
// MAD header and network byte order for rest of packet
void GsiSendInvalid(
	IN	IBT_DGRM_ELEMENT		*pDgrm
	)
{
	FSTATUS						status		= FSUCCESS;
	uint32						i;
	IBT_DGRM_ELEMENT			*pDgrmList;
	GSA_POST_SEND_LIST			*pPostList;
	MAD							*pMad;

	_DBG_ENTER_LVL(_DBG_LVL_SEND, GsiSendInvalid);

	i = 1;
	status = DgrmPoolGet( g_GsaGlobalInfo->DgrmPoolRecvQ, &i, &pDgrmList );
	if (FSUCCESS != status || i != 1)
	{
		_DBG_ERROR(("GsiSendInvalid: No more elements in Pool!!!\n"));
		goto done;
	}
	ASSERT(NULL != pDgrmList);

	// we are using a Recv Pool Dgrm
	pMad = GsiDgrmGetRecvMad(pDgrmList);

	// Copy MAD Headers from template
	MemoryCopy( pMad, GsiDgrmGetRecvMad(pDgrm), sizeof(MAD) );

	pMad->common.mr.AsReg8 = MMTHD_GET_RESP;
	pMad->common.u.NS.Status.AsReg16 = MAD_STATUS_UNSUPPORTED_CLASS_VER;
	pMad->common.u.NS.Reserved1 = 0;
	_DBG_INFO(("Invalid MAD: class %u\n", pMad->common.MgmtClass));
	// GsiDoSendDgrm will BSWAP_MAD_HEADER
			
	// Set remote info
	GsiDgrmAddrCopy(pDgrmList, pDgrm);

	// generate a PostSendList from the first datagram in post
	pPostList		= &(((IBT_DGRM_ELEMENT_PRIV *)pDgrmList)->PostSendList);

	// Fill in list details
	pPostList->DgrmIn = 1;
	AtomicWrite(&pPostList->DgrmOut, 0);
	pPostList->DgrmList			= pDgrmList;
	pPostList->SARRequest		= FALSE;

	status = GsiDoSendDgrm( NULL, pPostList);
	if (status != FSUCCESS)
	{
		// release AV
		DgrmPoolPut( pDgrmList );	
	}

done:
	_DBG_LEAVE_LVL(_DBG_LVL_SEND);
}
