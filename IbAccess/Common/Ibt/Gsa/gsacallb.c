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

#ifdef SA_ALLOC_DEBUG
extern IB_HANDLE sa_poolhandle;
#endif

static __inline void
RemoveClientIdFromTID(
		IN MAD_COMMON				*pMadHdr
		)
{
	if (MAD_IS_RESPONSE((MAD*)pMadHdr))
	{
		pMadHdr->TransactionID = (uint64)(pMadHdr->TransactionID
												& 0xffffffffffffff00ll);
	}
}

// called in a thread context (GsaEventThread), so can preempt
void 
GsaCompletionThreadCallback(
	IN	void				*CaContext
	)
{
	SMA_CA_OBJ_PRIVATE *pCaObj = (SMA_CA_OBJ_PRIVATE *)CaContext;

	GsaCompletionCallback( CaContext, pCaObj->Qp1CqHandle );
}

// called in a thread context, so can preempt
void 
GsaCompletionCallback(
	IN	void				*CaContext,
	IN	IB_HANDLE			CqHandle
	)
{
	FSTATUS					status;

	IBT_DGRM_ELEMENT		*pDgrm			= NULL;
	boolean					bRearmed		= FALSE;

	IBT_DGRM_ELEMENT_PRIV	*pDgrmPrivate;
	GSA_POST_SEND_LIST		*pPostList;
	MAD_COMMON				*pMadHdr;
	GSA_SERVICE_CLASS_INFO	*classInfo;
	IB_WORK_COMPLETION		workCompletion;
	uint8					clientId;
	uint64					portGuid;

	_DBG_ENTER_LVL(_DBG_LVL_CALLBACK, GsaCompletionCallback);
	//
	// Processing completions on the CQ is done as follows:
	//	1. Repeatedly Poll till all entries are done. 
	//	2. Rearm CQ and Poll again. If there are no more completions bail out.
	//	3. If there are completions go back to step 1.
	//	4. Skip step 2 and loop through 3.
	//
	//	We follow these steps to optimize on hardware events and race 
	//	conditions.
	//

	while (1)
	{
#ifdef SA_ALLOC_DEBUG
		if (iba_gsi_dgrm_pool_count(sa_poolhandle) < 50) MsgOut("GSA Callback\n");
#endif
		//
		// Loop and pull out all sends and receives
		//
		while ( FSUCCESS == (status = iba_poll_cq( CqHandle, &workCompletion )) )
		{
			// fill in common values
			pDgrm = (IBT_DGRM_ELEMENT *)((uintn)MASK_WORKREQID(workCompletion.WorkReqId));

			// This should never happen. test for bad hardware
			ASSERT ( pDgrm );
			if ( NULL == pDgrm )
			{
				//
				// This is for debug only. We should remove this conditional 
				// once the problem is debugged
				//
				_DBG_ERROR (("Spurious CQ Event!!!\n"
							"\tCompleted WQE.....:(%s)\n"
							"\tSTATUS............:x%x\n"
							"\tLength............:x%x\n",
							_DBG_PTR(((WCTypeSend == workCompletion.WcType) ? \
								"Send":"Recv")),
							workCompletion.Status,
							workCompletion.Length));

				if ( WCTypeRecv == workCompletion.WcType )
				{
					_DBG_ERROR (("\n"
							"\tQP Number.........:x%x\n"
							"\tSourceLID.........:x%x\n",
							workCompletion.Req.RecvUD.SrcQPNumber,
							workCompletion.Req.RecvUD.SrcLID));
				}

				_DBG_BREAK;
				break;
			}

			// process work completion
			pDgrm->Element.Status	= workCompletion.Status;
			pDgrmPrivate			= (IBT_DGRM_ELEMENT_PRIV *)pDgrm;
					
			_DBG_PRINT (_DBG_LVL_CALLBACK,(
				"Completed %s WQE (%p):\n"
				"\tSTATUS.......:x%X (%s)\n",
				_DBG_PTR(((WCTypeSend == workCompletion.WcType) ? "Send":"Recv")), 
				_DBG_PTR(pDgrm),
				workCompletion.Status, 
				_DBG_PTR(iba_wc_status_msg(workCompletion.Status)) ));

			if (IS_SQ_WORKREQID(workCompletion.WorkReqId))
			{
#ifdef SA_ALLOC_DEBUG
				if (iba_gsi_dgrm_pool_count(sa_poolhandle) < 50) MsgOut("GSA Send Compl\n");
#endif
			
				if (! pDgrm->Allocated) {
					_DBG_ERROR(("GSI ERROR: Send Completion for non-allocated Dgrm, discard: pDgrm=%p completion type=%d OnrecvQ=%d\n", _DBG_PTR(pDgrm), workCompletion.WcType, pDgrm->OnRecvQ));
					if (Gsa_debug_params.debug_level & _DBG_LVL_STORE)
					{
						ASSERT(pDgrm->Allocated);
					}
					DEBUG_ASSERT(pDgrm->Allocated);
					continue;
				}
				if (pDgrm->OnRecvQ) {
					_DBG_ERROR(("GSI ERROR: Send Completion for Recv Q Dgrm, discard: pDgrm=%p completion type=%d\n", _DBG_PTR(pDgrm), workCompletion.WcType));
					if (Gsa_debug_params.debug_level & _DBG_LVL_STORE)
					{
						ASSERT(! pDgrm->OnRecvQ);
					}
					DEBUG_ASSERT(! pDgrm->OnRecvQ);
					continue;
				}
				if (! pDgrm->OnSendQ) {
					_DBG_ERROR(("GSI ERROR: Send Completion for non-OnSendQ Dgrm, discard: pDgrm=%p completion type=%d allocated=%d\n", _DBG_PTR(pDgrm), workCompletion.WcType, pDgrm->Allocated));
					if (Gsa_debug_params.debug_level & _DBG_LVL_STORE)
					{
						ASSERT(pDgrm->OnSendQ);
					}
					DEBUG_ASSERT(pDgrm->OnSendQ);
					continue;
				}
				pDgrm->OnSendQ = FALSE;


				// Handle common send completion.  Only for SendInvalid
				// will a Recv Pool Dgrm be used, we get correct pMadHdr
				// mainly so log messages below are correct
				if (((IBT_DGRM_ELEMENT_PRIV *)pDgrm)->MemPoolHandle->ReceivePool )
					pMadHdr	= (MAD_COMMON *)GsiDgrmGetRecvMad(pDgrm);
				else
					pMadHdr	= (MAD_COMMON *)GsiDgrmGetSendMad(pDgrm);

				BSWAP_MAD_HEADER ((MAD*)pMadHdr);

				pPostList		= pDgrmPrivate->Base;
				AtomicIncrementVoid( &pPostList->DgrmOut);

#ifdef ICS_LOGGING
				_DBG_PRINT (_DBG_LVL_CALLBACK,(
						"Send Info:\n"
						"\tClass........:x%x\n"
						"\tResponseBit..:x%x\n"	
						"\tTID..........:x%"PRIx64"\n"
						"Local\n"
						"\tPathBits.....:x%X\n"
						"Remote:\n"
						"\tLID..........:x%X\n"
						"\tQP...........:x%X\n",
						(uint8)pMadHdr->MgmtClass, 
						pMadHdr->mr.s.R,
						pMadHdr->TransactionID,
						pDgrmPrivate->DgrmElement.PathBits,
						pDgrmPrivate->DgrmElement.RemoteLID,
						pDgrmPrivate->DgrmElement.RemoteQP
						));

				_DBG_PRINT (_DBG_LVL_CALLBACK,(
						"Send Info:\n"
						"\tClass........:x%x\n"
						"\tResponseBit..:x%x\n"	
						"\tTID..........:x%"PRIx64"\n"
						"Local\n"
						"\tQKey.........:x%X\n"
						"\tServiceLvl...:x%X\n"
						"\tGlobalRoute..:%s\n",
						(uint8)pMadHdr->MgmtClass, 
						pMadHdr->mr.s.R,
						pMadHdr->TransactionID,
						pDgrmPrivate->DgrmElement.RemoteQKey,
						pDgrmPrivate->DgrmElement.ServiceLevel,
						_DBG_PTR((pDgrmPrivate->DgrmElement.GlobalRoute	? "TRUE":"FALSE") )
						));

#else
				_DBG_PRINT (_DBG_LVL_CALLBACK,(
						"Send Info:\n"
						"\tClass........:x%x\n"
						"\tResponseBit..:x%x\n"	
						"\tTID..........:x%"PRIx64"\n"
						"Local\n"
						"\tPathBits.....:x%X\n"
						"Remote:\n"
						"\tLID..........:x%X\n"
						"\tQP...........:x%X\n"
						"\tQKey.........:x%X\n"
						"\tServiceLvl...:x%X\n"
						"\tRate.........:x%d\n"
						"\tGlobalRoute..:%s\n",
						(uint8)pMadHdr->MgmtClass, 
						pMadHdr->mr.s.R,
						pMadHdr->TransactionID,
						pDgrmPrivate->DgrmElement.PathBits,
						pDgrmPrivate->DgrmElement.RemoteLID,
						pDgrmPrivate->DgrmElement.RemoteQP,
						pDgrmPrivate->DgrmElement.RemoteQKey,
						pDgrmPrivate->DgrmElement.ServiceLevel,
						pDgrmPrivate->DgrmElement.StaticRate,
						(pDgrmPrivate->DgrmElement.GlobalRoute	? "TRUE":"FALSE") 
						));

#endif
				// Send completion for Send initiated by RMPP protocol
				if( pPostList->SARRequest)
				{
#ifdef SA_ALLOC_DEBUG
					if (iba_gsi_dgrm_pool_count(sa_poolhandle) < 50) MsgOut("GSA Send Compl: SAR\n");
#endif
					// send was posted by RMPP protocol handler
					// let it process the completion
					ProcessSARSendCompletion((MAD*)pMadHdr, pDgrm);
					continue;
				}
				
				// The datagram has been successfully sent.  Reset the
				// transaction ID to that specified by the user if this is
				// not a response.  Responses use the transaction ID from
				// the original request.
				if (MAD_IS_REQUEST((MAD*)pMadHdr))
				{
					// Restore the original TID from the user.  We cannot
					// use the context manager here, since we may have already
					// completed the request.  (The associated receive may
					// be processed before the send is completed.)
					pMadHdr->TransactionID = pDgrmPrivate->SavedSendTid;
				}

				// Free resources
				ASSERT ( pDgrmPrivate->AvHandle );
				status = PutAV( NULL, pDgrmPrivate->AvHandle );
				classInfo		= (GSA_SERVICE_CLASS_INFO*)pDgrmPrivate->MemPoolHandle->ServiceHandle;
				if ( NULL == classInfo )
				{
					// no associated class, must be a GSI initiated MAD
					DEBUG_ASSERT(pPostList->DgrmIn == AtomicRead(&pPostList->DgrmOut));
					status = DgrmPoolPut( pPostList->DgrmList );
					continue;
				}
				
				// Match all sends before callback
				if ( pPostList->DgrmIn != AtomicRead(&pPostList->DgrmOut) )
				{
#ifdef SA_ALLOC_DEBUG
					if (iba_gsi_dgrm_pool_count(sa_poolhandle) < 50) MsgOut("GSA Send Compl: more to do\n");
#endif
					_DBG_PRINT (_DBG_LVL_CALLBACK,(
						"All sends not completed in send list:\n"
						"\tTotal.....:%d\n"
						"\tDone......:%d\n",
						pPostList->DgrmIn,
						AtomicRead(&pPostList->DgrmOut) ));

					continue;
				}

				// Call user's send completion
#ifdef SA_ALLOC_DEBUG
				if (iba_gsi_dgrm_pool_count(sa_poolhandle) < 50) MsgOut("GSA Send Compl: class callback: %p\n", classInfo->SendCompletionCallback);
#endif
				if ( classInfo->SendCompletionCallback )
				{
					(classInfo->SendCompletionCallback)(
									classInfo->ServiceClassContext,
									pPostList->DgrmList );
				} else {
					status = DgrmPoolPut( pPostList->DgrmList );
				}	// Class Callback
			} else {
				// Receive Completion processing
				uint32 post_count;
				IBT_BUFFER *Current = NULL;
				uint32 Length = 0;
#ifdef SA_ALLOC_DEBUG
				if (iba_gsi_dgrm_pool_count(sa_poolhandle) < 50) MsgOut("GSA Recv Compl\n");
#endif
				if (pDgrm->OnSendQ) {
					_DBG_ERROR(("GSI ERROR: Recv Completion for Send Q Dgrm, discard: pDgrm=%p completion type=%d\n", _DBG_PTR(pDgrm), workCompletion.WcType));
					if (Gsa_debug_params.debug_level & _DBG_LVL_STORE)
					{
						ASSERT(! pDgrm->OnSendQ);
					}
					DEBUG_ASSERT(! pDgrm->OnSendQ);
					continue;
				}
				if (! pDgrm->OnRecvQ) {
					_DBG_ERROR(("GSI ERROR: Recv Completion for non-OnRecvQ Dgrm, discard: pDgrm=%p completion type=%d allocated=%d\n", _DBG_PTR(pDgrm), workCompletion.WcType, pDgrm->Allocated));
					if (Gsa_debug_params.debug_level & _DBG_LVL_STORE)
					{
						ASSERT(pDgrm->OnRecvQ);
					}
					DEBUG_ASSERT(pDgrm->OnRecvQ);
					continue;
				}

				if (! pDgrm->Allocated) {
					_DBG_ERROR(("GSI ERROR: Completion for non-allocated Dgrm, discard: pDgrm=%p completion type=%d\n", _DBG_PTR(pDgrm), workCompletion.WcType));
					if (Gsa_debug_params.debug_level & _DBG_LVL_STORE)
					{
						ASSERT(pDgrm->Allocated);
					}
					DEBUG_ASSERT(pDgrm->Allocated);
					pDgrm->OnRecvQ = FALSE;
					continue;
				}
				pDgrm->OnRecvQ = FALSE;


				status = DecrementGsiRecvQPostedForPortGuid(
												pDgrm->PortGuid, &post_count);
				_DBG_PRINT (_DBG_LVL_CALLBACK,("0x%"PRIx64": got post count of %u: %d\n",pDgrm->PortGuid, post_count, status));
				if (FSUCCESS == status
					&& post_count < g_GsaSettings.MinBuffersToPostPerPort
					)
				{
					// replenish to desired min
					post_count = g_GsaSettings.MinBuffersToPostPerPort - post_count;
				} else {
					// can't be sure, just repost 1
					post_count = 1;
				}
				_DBG_PRINT (_DBG_LVL_CALLBACK,("computed post count of %u\n", post_count));

				// Do not callback for QpFlush 
				if ( WRStatusWRFlushed == workCompletion.Status )
				{
#ifdef SA_ALLOC_DEBUG
				if (iba_gsi_dgrm_pool_count(sa_poolhandle) < 50) MsgOut("GSA Recv Compl: Flush\n");
#endif
					_DBG_PRINT (_DBG_LVL_CALLBACK,("WRFlushed: skip users.\n"));
					status = DgrmPoolPut( pDgrm );	
					continue;
				}
				/* MADs can be of variable length */
				if ( workCompletion.Length > (sizeof(IB_GRH) + sizeof(MAD)))
				{
					_DBG_ERROR ((
						"Invalid MAD receive length (%d bytes)! <Discard>\n",
						workCompletion.Length ));
					status = DgrmPoolPut( pDgrm );	
					continue;
				}
				// Overload base with SAR field
				pDgrmPrivate->Base = 0;		// not a SAR Receive Dgrm
				
				// Fill in bytes rcvd
				pDgrm->TotalRecvSize = workCompletion.Length;
				pDgrm->Element.RecvByteCount = workCompletion.Length;

				// Save GUID for a post since the user may FPENDING the Dgrm
				portGuid = pDgrm->PortGuid;

				// GRH is first buffer, MAD is second
				pDgrm->Element.pBufferList->ByteCount = sizeof(IB_GRH);

				Current = pDgrm->Element.pBufferList->pNextBuffer;
				Length = workCompletion.Length - sizeof(IB_GRH);

				pMadHdr = (MAD_COMMON *)GsiDgrmGetRecvMad(pDgrm);

				while (Length && Current)
				{
					Current->ByteCount = (pMadHdr->BaseVersion == IB_BASE_VERSION)
						? MIN(Length, IB_MAD_BLOCK_SIZE)
						: MIN(Length, STL_MAD_BLOCK_SIZE);

					if (__DBG_LEVEL__ & _DBG_LVL_PKTDUMP)
					{
						_DBG_DUMP_MEMORY(_DBG_LVL_PKTDUMP, ("Recv GSI MAD Packet"),
								Current->pData,
								MIN(Current->ByteCount, Length));
					}

					Length -= Current->ByteCount;
					Current = Current->pNextBuffer;
				}

				BSWAP_MAD_HEADER ((MAD*)pMadHdr);
#ifdef ICS_LOGGING
				_DBG_PRINT (_DBG_LVL_CALLBACK,(
						"Receive Info:\n"
						"\tClass........:x%x\n"
						"\tResponseBit..:x%x\n"
						"\tTID..........:x%"PRIx64"\n"
						"Local\n"
						"\tPathBits.....:x%X\n"
						"Remote:\n"
						"\tLID..........:x%X\n"
						"\tQP...........:x%X\n",
						(uint8)pMadHdr->MgmtClass, 
						pMadHdr->mr.s.R,
						pMadHdr->TransactionID,
						workCompletion.Req.RecvUD.DestPathBits,
						workCompletion.Req.RecvUD.SrcLID,
						workCompletion.Req.RecvUD.SrcQPNumber
						));
				_DBG_PRINT (_DBG_LVL_CALLBACK,(
						"Receive Info:\n"
						"\tClass........:x%x\n"
						"\tResponseBit..:x%x\n"
						"\tTID..........:x%"PRIx64"\n"
						"Remote:\n"
						"\tPKeyIndex....:x%X\n"
						"\tServiceLvl...:x%X\n"
						"\tGlobalRoute..:%s\n",
						(uint8)pMadHdr->MgmtClass, 
						pMadHdr->mr.s.R,
						pMadHdr->TransactionID,
						workCompletion.Req.RecvUD.PkeyIndex,
						workCompletion.Req.RecvUD.ServiceLevel,
						_DBG_PTR((workCompletion.Req.RecvUD.Flags.s.GlobalRoute ? "TRUE":"FALSE"))
						));
				
#else
				_DBG_PRINT (_DBG_LVL_CALLBACK,(
						"Receive Info:\n"
						"\tClass........:x%x\n"
						"\tResponseBit..:x%x\n"
						"\tTID..........:x%"PRIx64"\n"
						"Local\n"
						"\tPathBits.....:x%X\n"
						"Remote:\n"
						"\tLID..........:x%X\n"
						"\tQP...........:x%X\n"
						"\tPKeyIndex....:x%X\n"
						"\tServiceLvl...:x%X\n"
						"\tGlobalRoute..:%s\n",
						(uint8)pMadHdr->MgmtClass, 
						pMadHdr->mr.s.R,
						pMadHdr->TransactionID,
						workCompletion.Req.RecvUD.DestPathBits,
						workCompletion.Req.RecvUD.SrcLID,
						workCompletion.Req.RecvUD.SrcQPNumber,
						workCompletion.Req.RecvUD.PkeyIndex,
						workCompletion.Req.RecvUD.ServiceLevel,
						(workCompletion.Req.RecvUD.Flags.s.GlobalRoute ? "TRUE":"FALSE")
						));

#endif
				// Port Number and Q pair are filled-up during 
				// Gsa's PostRecv
				pDgrm->PathBits		= workCompletion.Req.RecvUD.DestPathBits;
				pDgrm->RemoteLID	= workCompletion.Req.RecvUD.SrcLID;
				pDgrm->RemoteQP		= workCompletion.Req.RecvUD.SrcQPNumber;
				pDgrm->RemoteQKey	= QP1_WELL_KNOWN_Q_KEY;
				pDgrm->PkeyIndex	= workCompletion.Req.RecvUD.PkeyIndex;
				pDgrm->ServiceLevel	= workCompletion.Req.RecvUD.ServiceLevel;
				// IB does not have Static rate in UD LRH
				// so we can't be sure what rate of remote port is
				// we use a constant value for GSI QPs
				pDgrm->StaticRate	= IB_STATIC_RATE_GSI;
				pDgrm->GlobalRoute	= workCompletion.Req.RecvUD.Flags.s.GlobalRoute;

				if (workCompletion.Req.RecvUD.Flags.s.GlobalRoute)
				{
					IB_GRH *pGrh = GsiDgrmGetRecvGrh(pDgrm);
					BSWAP_IB_GRH ( pGrh);
					pDgrm->GRHInfo.DestGID = pGrh->SGID;
					pDgrm->GRHInfo.FlowLabel = pGrh->u1.s.FlowLabel;
					pDgrm->GRHInfo.SrcGIDIndex = 0; // BUGBUG - lookup DGID
					pDgrm->GRHInfo.HopLimit = pGrh->HopLimit;
					pDgrm->GRHInfo.TrafficClass = pGrh->u1.s.TClass;

					_DBG_PRINT (_DBG_LVL_CALLBACK,(
						"GRH Info:\n"
						"\tDGID.........:x%"PRIx64":%"PRIx64"\n"
						"\tSGID.........:x%"PRIx64":%"PRIx64"\n"
						"\tPayload......:%d bytes\n",
						pGrh->DGID.Type.Global.SubnetPrefix,
						pGrh->DGID.Type.Global.InterfaceID,
						pGrh->SGID.Type.Global.SubnetPrefix,
						pGrh->SGID.Type.Global.InterfaceID,
						pGrh->PayLen ));
				}

				//
				//TBD:
				// Chain replies to process later
				//
				//pDgrm->pNextCompDgrm
			
				// For RMPP R bit be set in all packets which compose a response
				// message  Even ABORT, STOP, etc.
				// transaction ID will be consistent in all packets for
				// a given request/respone message pair.
				// R bit be 0 in all packets which compose a request message
				// event ACK, ABORT, STOP, etc

				// See if the received MAD was in response to a previously
				// sent MAD request.
                if (MAD_IS_RESPONSE((MAD*)pMadHdr))
				{
					// Response to clients.  client id was a field
					// in the transaction ID sent on wire
					clientId = (uint8)(pMadHdr->TransactionID & 0xff);
				} else {
					// This is a new request or not a request/response pair
					// TID unmodified TID is visible to client (for a request
					// TID was remotely generated and we can't modify)
					clientId = MAX_CLIENT_ID;
					status = FSUCCESS;
				}

				// Validate Attributes to be processed
				if( status == FSUCCESS )
				{
					// Does the Client want us to do RMPP protocol for it?
					if( IsSarClient( clientId, pMadHdr->MgmtClass ) )
					{
						// Process the segment.  If all segments have been
						// received, pDgrm will be updated to point to the
						// first segment.
						status = ProcessSARRecvCompletion((MAD*)pMadHdr,&pDgrm);
						if( status == FCOMPLETED )
						{
							// remove clientid from all TIDs before give to user
							IBT_ELEMENT *pElement;
							for (pElement = &pDgrm->Element; pElement != NULL; pElement = pElement->pNextElement)
							{
								// first buffer is GRH, 2nd is start of MAD
								RemoveClientIdFromTID(
									(MAD_COMMON *)
									pElement->pBufferList->pNextBuffer->pData);
							}

							// adjust pMadHdr to be first in list of Dgrms,
							// pDgrm could have been changed by SAR processing
							pMadHdr = (MAD_COMMON *)GsiDgrmGetRecvMad(pDgrm);
						}
					} else {
						// No SAR is needed.

						// remove clientid from TID before give to user
						RemoveClientIdFromTID(pMadHdr);
						status = FCOMPLETED;
					}

					// Notify the user of fully received MADs.
					if( status == FCOMPLETED )
					{
						status = GsaNotifyClients( clientId, pMadHdr, pDgrm );
					}

					if (status == FNOT_FOUND)
					{
						// no client found
						if (MAD_EXPECT_GET_RESP((MAD*)pMadHdr))
						{
							// reply with a bad class version response
							GsiSendInvalid(pDgrm);
						}
						status = FREJECT;	// free dgrm below
					}
					// Free the datagram(s) if not required for SAR and not
					// being used by the client.
					if( status == FCOMPLETED || status == FREJECT )
					{
						DgrmPoolPut( pDgrm );
					}
				}

				// Post back a recv to Q 
				status = PostRecvMsgToPort( post_count, portGuid );
			}			// recv
		}				// while loop

		if ( FNOT_DONE != status )
		{
			_DBG_ERROR (("Got %s Status!!!\n", _DBG_PTR(FSTATUS_MSG(status)) ));
		}
		
		// Rearm CQ for race conditions
		if ( FALSE == bRearmed )
		{
			status = iba_rearm_cq( CqHandle, CQEventSelNextWC );						
			bRearmed = TRUE;
		} else {
			break;
		}
	}	// while 1

    _DBG_LEAVE_LVL(_DBG_LVL_CALLBACK);
}
