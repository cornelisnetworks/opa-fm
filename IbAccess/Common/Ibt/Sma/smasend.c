/* BEGIN_ICS_COPYRIGHT4 ****************************************

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

** END_ICS_COPYRIGHT4   ****************************************/

#include "smamain.h"




//
// SmaPostSend
//
//	All SMPs are sent out by the SM with this call. The caller can send one 
//	to many SMPs by listing each SMP through a linked list of SMP_BLOCKs. 
//	The user will specify the total SMPs contained in the list.
//
//
// INPUTS:
//
//	SmObject		-Object returned in the Open call.
//
//	NumSmps			-Number of SMPs to send
//
//	SmpBlockList	-Linked list of send requests contained in SMP_BLOCKs
//
//	PostCallback	-A callback for SMPs Posted through SmpBlockList
//
//	PostContext		-user context that will be passed back on a post callback
//
//
// OUTPUTS:
//
//
// RETURNS:
//
//
// IRQL:
//
//

FSTATUS
iba_smi_post_send(
	IN	SMA_OBJECT			*SmObject,
	IN	uint32				NumSmps,
	IN	SMP_BLOCK			*SmpBlockList,
	IN	SM_POST_CALLBACK	*PostCallback,
	IN	void				*PostContext
	)
{
	FSTATUS					status			= FSUCCESS;
	boolean					bLocal			= TRUE;

	uint32					smps, tmpSmps;
	POOL_DATA_BLOCK			*pReq, *pSmpBlock;
	SMA_CA_OBJ_PRIVATE		*pCaObj;
	SMA_PORT_TABLE_PRIV		*pPortTbl;
	SMP_LIST				*pSmpList;
	STL_SMP					*pSmp;
	uint8					portNo;
		

	_DBG_ENTER_LVL ( _DBG_LVL_SEND, iba_smi_post_send );
	

	// Check params
	smps					= 0;
	pReq					= (POOL_DATA_BLOCK *)SmpBlockList;

	while ( smps < NumSmps ) 
	{
		pSmp				= (STL_SMP*)pReq->Block.Smp;

		if (pReq->Block.SmpByteCount == 0)
		{
			if (pSmp->common.BaseVersion == IB_BASE_VERSION)
				pReq->Block.SmpByteCount = IB_MAD_BLOCK_SIZE;
			else
				pReq->Block.SmpByteCount = STL_MAD_BLOCK_SIZE;
		}
		
		// Get CaContext
		pCaObj				= (SMA_CA_OBJ_PRIVATE *)pReq->Block.SmaCaContext;

		if ( NULL != pCaObj )
		{

			pPortTbl		= (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;

			// Check if Port number is valid
			if (pCaObj->CaPublic.Capabilities & CA_IS_ENHANCED_SWITCH_PORT0)
			{
				if ( pReq->Block.PortNumber != 0 )
				{
					// Error return, free buffers...
					_DBG_ERROR (( "Invalid Port!\n" ));
					pReq->Block.Status = status = FINVALID_PARAMETER;
					break;
				}
				portNo = 0;
			}
			else if ( pReq->Block.PortNumber < 1
				|| pReq->Block.PortNumber > pCaObj->CaPublic.Ports )
			{
				// Error return, free buffers...
				_DBG_ERROR (( "Invalid Port!\n" ));
				pReq->Block.Status = status = FINVALID_PARAMETER;
				break;
			}
			else
			{
				portNo = pReq->Block.PortNumber - 1;
			}

			if ( QP_DESTROY == \
				pPortTbl[portNo].PortBlock->Qp0Control )
			{
				_DBG_ERROR (( "Qp set to Destroy state!\n" ));
				pReq->Block.Status = status = FINVALID_STATE;
				break;
			}	
			else
			{
				// check SLID, 0 is invalid also used as special internal value
				if ( 0 == pReq->Block.SLID)
				{
					_DBG_ERROR (( "SLID 0 !\n" ));
					pReq->Block.Status = status = FINVALID_PARAMETER;
					break;
				}
				// Do some sanity checks for outgoing SMP from SM
				if ( 1 == pSmp->common.mr.s.R ) 
				{
					// Only SM_INFO can be passed up from SM as a Response
					if ( MCLASS_ATTRIB_ID_SM_INFO != pSmp->common.AttributeID )
					{
						_DBG_ERROR (( "R bit set!\n" ));
						pReq->Block.Status = status = FINVALID_PARAMETER;
						break;
					}

					// Validate D bit, HopPointer and HopCount
					// is per IBTA 1.1 sec 14.2.2.3
                    if ( MCLASS_SM_DIRECTED_ROUTE == pSmp->common.MgmtClass )
                    {
						// D bit should be set
                        if ( 0 == pSmp->common.u.DR.s.D )
                        {
                            _DBG_ERROR ((
                                "Status D bit mismatch!\n" ));
                            pReq->Block.Status = status = FINVALID_PARAMETER;
                            break;
                        }

#if 0	// tests may not be valid if we use this interface to SMINFO ourselves
						// HopCount cannot be zero at SM level in a response
						if (0 == pSmp->common.u.DR.HopCount)
						{
                            _DBG_ERROR ((
                                "HopCount zero response not available to SM!\n" ));
                            pReq->Block.Status = status = FINVALID_PARAMETER;
                            break;
						}

						// HopPointer must be equal to HopCount + 1
						if (0 != pSmp->common.u.DR.HopCount)
                        {
                            if ( pSmp->common.u.DR.HopPointer !=
                                 (pSmp->common.u.DR.HopCount+1))
                            {
                                _DBG_ERROR ((
                                    "(HopPointer != HopCount+1) mismatch!\n" ));
                                pReq->Block.Status = status = FINVALID_PARAMETER;
                                break;
                            }
                        }	// HC check
#endif
                    }		// DR Smp check
				}			// R bit == 1 check
				else
				{
					if ( MCLASS_SM_DIRECTED_ROUTE == pSmp->common.MgmtClass )
					{
						// validate per IBTA 1.1 sec 14.2.2.1
						// D bit should be zero & HP should be zero
						if ( 0 != ( pSmp->common.u.DR.s.D | \
									pSmp->common.u.DR.HopPointer )) 
						{
							_DBG_ERROR (( 
								"Status D bit or HopPointer non-zero mismatch!\n" ));
							pReq->Block.Status = status = FINVALID_PARAMETER;
							break;
						}
						
						// Local loopback check
						if (( 0 == pSmp->common.u.DR.HopCount ) && \
							( pReq->Block.DLID != STL_LID_PERMISSIVE ))
						{
							_DBG_ERROR ((
								"Local loopback with DR DLID not Permissive!\n" ));
							pReq->Block.Status = status = FINVALID_PARAMETER;
							break;
						}	
					}		// DR Smp check
				}			// R bit == 0 check
			}				// Port check
		}
		else
		{
			_DBG_ERROR ((
				"Bad CaContext!!!\n" ));
			pReq->Block.Status = status = FINVALID_PARAMETER;
			break;
		}


		// We need to trigger on a need basis to optimize interrupts.
		// Look up port out usage
		pReq->CQTrigger		= TRUE;

		// Do all post requests
		smps++;
		pReq				= (POOL_DATA_BLOCK*)pReq->Block.Next;

	}	// Query all smps to be posted
	

	// Only proceed if success
	if ( FSUCCESS == status )
	{
		// Load Smp list
		smps					= 0;
		pSmpList = (SMP_LIST*)MemoryAllocateAndClear(sizeof(SMP_LIST), FALSE, SMA_MEM_TAG);

		if (pSmpList == NULL) {
			status = FINSUFFICIENT_MEMORY;
			goto exit;
		}

		// Prepare Posts and send out
		pSmpList->SmpsIn		= NumSmps;			// input
		AtomicWrite(&pSmpList->SmpsOut, 0);			// output
		pSmpList->SmpsInError	= 0;				// errors
		pSmpList->SmpBlock		= SmpBlockList;
		pSmpList->PostCallback	= PostCallback;
		pSmpList->Context		= PostContext;

		// walk through list
		pReq					= (POOL_DATA_BLOCK *)SmpBlockList;
		while ( smps < NumSmps ) 
		{
			SMI_ACTION SmiAction;

			// link the list for active lookup in completions
			pReq->Base			= pSmpList;			
			pReq->Block.Status	= 0;

			pSmp				= (STL_SMP*)pReq->Block.Smp;
			
			// Get CaContext
			pCaObj			= (SMA_CA_OBJ_PRIVATE *)pReq->Block.SmaCaContext;
			pPortTbl		= (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;

			// Clear out usage fields
			pReq->AvHandle	= 0;

			SmiAction = SmaProcessSmiSmp(pCaObj, pReq, FALSE);

			if (pCaObj->CaPublic.Capabilities & CA_IS_ENHANCED_SWITCH_PORT0)
			{
				portNo = 0;
			}
			else
			{
				portNo = pReq->Block.PortNumber - 1;
			}

			// LID test below is not perfect since LMC should be included
			// in computation.  However if its wrong and we put on wire
			// hardware should post back to us anyway, so not a big deal
			if (SMI_TO_LOCAL == SmiAction 
				|| SMI_TO_LOCAL_SM == SmiAction
				|| (SMI_LID_ROUTED == SmiAction
					&& pReq->Block.DLID ==
							(pPortTbl[portNo].PortBlock->Public.Address.BaseLID | pReq->Block.PathBits)
					)
				)
			{
				// Local loopback
				tmpSmps		= 1;

				// grab a memory block
				status = iba_smi_pool_get(
							SmObject,
							&tmpSmps,			
							(SMP_BLOCK **)&pSmpBlock			
							);

				if ( FSUCCESS == status )
				{
					// Call internally and post on Recvd Q
					MemoryCopy(
							pSmpBlock->Block.Smp,
							pReq->Block.Smp,
							sizeof(STL_SMP)
							);

					// call common local function
					pSmpBlock->Block.SmaCaContext	= pReq->Block.SmaCaContext;
					pSmpBlock->Block.PortNumber		= pReq->Block.PortNumber;
					pSmpBlock->Block.SmpByteCount	= pReq->Block.SmpByteCount;
					pSmpBlock->Block.SLID			= pReq->Block.SLID;
					pSmpBlock->Block.DLID			= pReq->Block.DLID;
					pSmpBlock->Block.PathBits		= pReq->Block.PathBits;
					pSmpBlock->Block.ServiceLevel	= pReq->Block.ServiceLevel;
					pSmpBlock->Block.StaticRate		= pReq->Block.StaticRate;
					

					if (SMI_TO_LOCAL == SmiAction || SMI_LID_ROUTED == SmiAction)
					{
						SmiAction = SmaProcessLocalSmp(pCaObj, pSmpBlock);
					}
					if (SMI_DISCARD == SmiAction)
					{
						(void)iba_smi_pool_put(
											SmObject,
											(SMP_BLOCK *)pSmpBlock);
					} else {
						// assume any SMI_TO_LOCAL, SMI_TO_LOCAL_SM or SMI_SEND
						// or SMI_LID_ROUTED response
						// needs to go back to the requestor
						// so dump block in recv
						SmaAddToRecvQ(pSmpBlock);
					}
				}
				else
				{
					pReq->Block.Status = FINSUFFICIENT_MEMORY;
					status = FSUCCESS;
				}

				// Add to SendQ (eg. PostSend send completions)
				SmaAddToSendQ( pReq );
			}
			else if ((SMI_SEND == SmiAction)
				|| (SMI_LID_ROUTED == SmiAction))
			{
				// Set local callback off
				bLocal = FALSE;

				status = SmaPostSmp(
								pCaObj,
								pReq );
				if (FSUCCESS != status)
				{
					// discarded - unable to send
					// Add to SendQ (eg. PostSend send completions)
					SmaAddToSendQ( pReq );
					status = FSUCCESS;
				}
			} else {
				// discarded
				// Add to SendQ (eg. PostSend send completions)
				SmaAddToSendQ( pReq );
			}
		
		
			// Do all post requests
			smps++;
			pReq	= (POOL_DATA_BLOCK*)pReq->Block.Next;

		}	// Post all smps


		// Add Smp block to global list
		// Lock SmpList
		// unlock block

		// If only local send do a manual recv completions 
		if ( TRUE == bLocal )
		{
			SmaCompletionCallback( NULL );
		}
	}

exit:
	_DBG_LEAVE_LVL(_DBG_LVL_SEND);
	return status;
}

//
// SmaPostSmp
//
//	All SMPs Sent on the wire go through this call. This call is used by
//	SmaPostSend() and Internal Callback reply sends as well as directed route
//	sends
//
// INPUTS:
//
//	CaObj			-CA on which the SMP goes out
//	Req				-The SMP request being sent out
//	CqTrigger		-Do a ream of CQ if TRUE
//
//
// OUTPUTS:
//
//
//
// RETURNS:
//
//
// IRQL:
//
//

FSTATUS
SmaPostSmp(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj,
	IN	POOL_DATA_BLOCK			*Req
	)
{
	FSTATUS						status;
	IB_WORK_REQ					*workRequest;
	CA_MEM_LIST					caMemList;
	SMA_PORT_TABLE_PRIV			*pPortTbl;
	STL_SMP						*pSmp;
	IB_WORK_REQ					wrq;
	IB_LOCAL_DATASEGMENT		dsList[1];
	IB_ADDRESS_VECTOR			avInfo;
	uint8						portNo;

			

	_DBG_ENTER_LVL ( _DBG_LVL_SEND, SmaPostSmp );

	
	MemoryClear(&wrq, sizeof(wrq));

	workRequest					= &wrq;
	workRequest->DSList			= &dsList[0];

	pSmp						= (STL_SMP*)Req->Block.Smp;

	
#ifdef IB_DEBUG
	//
	// Trap Debug Info
	//
	if ( MMTHD_TRAP == pSmp->common.mr.s.Method )
	{
	  BSWAP_MAD_HEADER( (MAD*)pSmp);
	  BSWAP_IB_NOTICE(((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData)));

#ifdef ICS_LOGGING
	  _DBG_PRINT(_DBG_LVL_CALLBACK, (
				">>>>Trap Info:>>>>>\n"
				"\tAttribModifier....: %x\n"
				"\tStatus............: %x\n"
				"\tIsGenericBit......: %x\n"
				"\tTrapNumber........: %x\n"
				"\tProducerType......: %x\n"
				"\tType..............: %x\n",
				pSmp->common.AttributeModifier,
				pSmp->common.u.NS.Status.AsReg16,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->u.Generic.u.s.IsGeneric,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->u.Generic.TrapNumber,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->u.Generic.u.s.ProducerType,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->u.Generic.u.s.Type
				));
	  _DBG_PRINT(_DBG_LVL_CALLBACK, (
				">>>>Trap Info (cont):>>>>>\n"
				"\tIssuerLID.........: %x\n"
				"\tNoticeCount.......: %x\n"
				"\tNoticeToggle......: %x\n"
				"\tdata..............: %x\n",
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->IssuerLID,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->Stats.Count,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->Stats.Toggle,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->Data[0]							
				));


#else
		_DBG_PRINT (_DBG_LVL_CALLBACK, (
				">>>>Trap Info:>>>>>\n"
				"\tAttribModifier....: %x\n"
				"\tStatus............: %x\n"
				"\tIsGenericBit......: %x\n"
				"\tTrapNumber........: %x\n"
				"\tProducerType......: %x\n"
				"\tType..............: %x\n"
				"\tIssuerLID.........: %x\n"
				"\tNoticeCount.......: %x\n"
				"\tNoticeToggle......: %x\n"
				"\tdata..............: %x\n",
				pSmp->common.AttributeModifier,
				pSmp->common.u.NS.Status.AsReg16,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->u.Generic.u.s.IsGeneric,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->u.Generic.TrapNumber,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->u.Generic.u.s.ProducerType,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->u.Generic.u.s.Type,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->IssuerLID,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->Stats.Count,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->Stats.Toggle,
				((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData))->Data[0]								
				));
#endif
#undef ProducerType
	  BSWAP_IB_NOTICE(((IB_NOTICE *)(pSmp->SmpExt.LIDRouted.SMPData)));
	  BSWAP_MAD_HEADER( (MAD*)pSmp);
	}	// trap info
#endif /* IB_DEBUG */

	//
	// get end-point info
	//
	pPortTbl = (SMA_PORT_TABLE_PRIV *)CaObj->CaPublic.PortTbl;

    if (CaObj->CaPublic.Capabilities & CA_IS_ENHANCED_SWITCH_PORT0)
    {
        portNo = 0;
    }
    else
    {
        portNo = Req->Block.PortNumber - 1;
    }

	avInfo.PortGUID				= pPortTbl->PortBlock[portNo].Public.GUID;
	avInfo.DestLID				= Req->Block.DLID;
	avInfo.PathBits				= Req->Block.PathBits;
	avInfo.ServiceLevel			= Req->Block.ServiceLevel;
	avInfo.StaticRate			= Req->Block.StaticRate;
	avInfo.GlobalRoute			= FALSE;
	avInfo.GlobalRouteInfo.FlowLabel			= 0;

	status  = GetAV( 
				CaObj,
				&avInfo,
				&Req->AvHandle );
				

	if ( FSUCCESS == status )
	{

		//
		// prepare the request
		//

		workRequest->Operation			= WROpSend;

		workRequest->DSList[0].Address	= (uintn)Req->Block.Smp;
		workRequest->DSList[0].Length	= Req->Block.SmpByteCount;

		GetMemInfoFromCaMemTable(
					CaObj,
					Req->CaMemIndex,
					&caMemList);

		workRequest->DSList[0].Lkey		= caMemList.LKey;

		workRequest->DSListDepth		= 1;
		workRequest->MessageLen			= Req->Block.SmpByteCount;
		
		workRequest->Req.SendUD.AVHandle= Req->AvHandle;

		//
		//workRequest->Req.SendUD.ImmediateData = 0xdeadbeef;
		//
		workRequest->Req.SendUD.Options.AsUINT16			= 0;
		workRequest->Req.SendUD.Options.s.SignaledCompletion= 1;
#if INCLUDE_16B
		workRequest->Req.SendUD.Options.s.SendFMH			= 0;
#endif
		
		//
		// Shall be set to zero for SMPs!
		//
		workRequest->Req.SendUD.QPNumber					= 0;
		workRequest->Req.SendUD.Qkey						= 0;				
		
		//
		// set Id to request stucture to unload
		//
		ASSERT_VALID_WORKREQID((uintn)Req);
		workRequest->WorkReqId		= BUILD_SQ_WORKREQID((uintn)Req);
		CaObj->WorkReqId++;

		//
		// Byte Order MAD Commen Header
		//
		BSWAP_STL_SMP_HEADER ((STL_SMP*)Req->Block.Smp);

#if 0
		//
		// yank request sends from pre-existing blocks.
		// if they do not exist, create one. if the requirement
		// is high, the new creation will satisfy the request
		//WorkReq = SmObject->WorkReqSend;
		//
		if ( TRUE == Req->CQTrigger )
		{

			//
			// The next Work Completion written to the 
			// CQ will generate an event
			//
			
			status = iba_rearm_cq(
				CaObj->CqHandle,
				CQEventSelNextWC		
				);
		}
#endif

		SpinLockAcquire( &pPortTbl[portNo].PortBlock->Qp0Lock );

		status = iba_post_send( 
					pPortTbl[portNo].PortBlock->QpHandle,
					workRequest );

		SpinLockRelease( &pPortTbl[portNo].PortBlock->Qp0Lock );

	}
	else
	{
		_DBG_ERROR ((
			"AV error on LID = x%x! \n", 
			avInfo.DestLID ));

	}	// AV handle 

	if ( FSUCCESS != status )
	{
		Req->Block.Status = status;
	}

	_DBG_LEAVE_LVL (_DBG_LVL_SEND);
	return status;
}
