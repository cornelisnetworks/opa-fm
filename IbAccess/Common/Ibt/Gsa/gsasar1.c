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

#if ! IB1_1

// 1.0a RMPP protocal Segmentation And Reassembly

//
// Definitions
//

//#define TEST_SAR_RECV_DISCARD 7	// enables discard for testing of retries/resend
#undef TEST_SAR_RECV_DISCARD

#define	GSA_SAR_MAX_RETRIES_SEND			10
#define	GSA_SAR_MAX_RETRIES_RECV			1

#define GSA_LARGEST_SAR_BLOCK				32 * 1024
#define	GSA_SAR_TIME_SEND					1000 * 2 

#if defined(DBG) || defined(IB_DEBUG)

#define	GSA_SAR_TIME_RECV					300000

#else

#define	GSA_SAR_TIME_RECV					1250

#endif

#ifdef ICS_LOGGING
#define _DBG_PRINT_SAR_HEADERS( LEVEL, TITLE, pSAR_MAD, REMOTE_LID) \
	do { \
	_DBG_PRINT (LEVEL, ( \
		TITLE 								\
		"\tClass..........:x%x\n"			\
		"\tRemoteLID......:x%x\n"			\
		"\tSegment........:%d\n"			\
		"\tPayloadLen.....:%d bytes\n",		\
		pSAR_MAD->common.MgmtClass, 	\
		REMOTE_LID,							\
		pSAR_MAD->Sar.SegmentNum,			\
		pSAR_MAD->Sar.PayloadLen			\
		));													\
	_DBG_PRINT (LEVEL, ( \
		TITLE 								\
		"\tFragmentFlag...:\n"				\
		"\t\tFirstPkt..:x%x\n"				\
		"\t\tLastPkt...:x%x\n"				\
		"\t\tKeepAlive.:x%x\n"				\
		"\t\tResendReq.:x%x\n"				\
		"\t\tAck.......:x%x\n",				\
		(pSAR_MAD->Sar.FragmentFlag.F.FirstPkt),		\
		(pSAR_MAD->Sar.FragmentFlag.F.LastPkt),		\
		(pSAR_MAD->Sar.FragmentFlag.F.KeepAlive),	\
		(pSAR_MAD->Sar.FragmentFlag.F.ResendReq),	\
		(pSAR_MAD->Sar.FragmentFlag.F.Ack)			\
		));									\
	} while (0)
#else
#define _DBG_PRINT_SAR_HEADERS( LEVEL, TITLE, pSAR_MAD, REMOTE_LID) \
	_DBG_PRINT (LEVEL, ( \
		TITLE 								\
		"\tClass..........:x%x\n"			\
		"\tRemoteLID......:x%x\n"			\
		"\tSegment........:%d\n"			\
		"\tPayloadLen.....:%d bytes\n"		\
		"\tFragmentFlag...:\n"				\
		"\t\tFirstPkt..:x%x\n"				\
		"\t\tLastPkt...:x%x\n"				\
		"\t\tKeepAlive.:x%x\n"				\
		"\t\tResendReq.:x%x\n"				\
		"\t\tAck.......:x%x\n",				\
		(uint8)pSAR_MAD->common.MgmtClass, 	\
		REMOTE_LID,							\
		pSAR_MAD->Sar.SegmentNum,			\
		pSAR_MAD->Sar.PayloadLen,			\
		(uint8)(pSAR_MAD->Sar.FragmentFlag.F.FirstPkt),		\
		(uint8)(pSAR_MAD->Sar.FragmentFlag.F.LastPkt),		\
		(uint8)(pSAR_MAD->Sar.FragmentFlag.F.KeepAlive),	\
		(uint8)(pSAR_MAD->Sar.FragmentFlag.F.ResendReq),	\
		(uint8)(pSAR_MAD->Sar.FragmentFlag.F.Ack)			\
		))
#endif

// RMPP Sender/Receiver Context
// for Segmentation and Reassembly of IBTA 1.0a RMPP messages
typedef struct _GSA_SAR_CONTEXT {
	
	// Common info
	LIST_ITEM						ListItem;
	SPIN_LOCK						Lock;		// lock

	uint32							RetryCount;
	boolean							ResendSent;	// Resend request has been sent

	uint32							TotalSegs;	// total no. of segments
	uint32							CurrSeg;	// current seg. sent
	uint32							TotalDataSize;	// size of data
	uint16							Window;		// Timeout value for multipacket MAD

	TIMER							TimerObj;
	uint64							TID;		// Client TID
	SYS_CALLBACK_ITEM 				*SysCBItem;	// Syscallback item
	uint8							MgmtClass;
	
	// recv info
	IBT_DGRM_ELEMENT				*RecvHead;
	IBT_DGRM_ELEMENT				*RecvTail;
	uint16							NextWindow;	// Timeout value for multipacket MAD
	

	// send info
	GSA_POST_SEND_LIST				*PostList;
	IB_HANDLE						Qp1Handle;	
	SPIN_LOCK						*QpLock;

	IB_L_KEY						LKey;
	IB_WORK_REQ						WorkRequest;

	IB_LOCAL_DATASEGMENT			*DataSeg;	// pointer to data segment
	uintn							DataStart;	// start of data


} GSA_SAR_CONTEXT;

//
// Function prototypes.
//
// ---------------------------------------------------------------------------
// RMPP Sender
FSTATUS
SARPost(
	IN	GSA_SAR_CONTEXT		*pSarContext
	);

boolean
ResendReqSARSend ( 
	IN	MAD_SAR					*pMadSar,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	);

boolean
AckReqSARSend( 
	IN	MAD_SAR					*pMadSar,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	);

void
SARCloseSend(
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN	boolean					callbackUser
	);

void
TimerCallbackSend(
	IN	void					*TimerContext
	);

// ---------------------------------------------------------------------------
// RMPP Receiver

boolean
IsRmppMethod(
	IN MAD_SAR					*pMadSarHdr
	);

FSTATUS
CreateSARRecv(
	IN	IBT_DGRM_ELEMENT		*pDgrm
	);

FSTATUS
SARRecv(
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	);

void
SARSendSegments(
	IN	void					*Key,
	IN	void					*TimerContext
	);

void
FragmentOptionsSARRecv (
	IN	IBT_DGRM_ELEMENT		*pDgrm,
	IN	FRAGMENT_FLAG			FragmentFlag,
	IN	uint64					TransactionID,
	IN	uint32					SegmentNum
	);

boolean
KeepAliveSARRecv ( 
	IN	MAD_SAR					*pMadSar,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	);

void
SendAckSARRecv ( 
	IN	IBT_DGRM_ELEMENT		*pDgrm
	);

void
ResendReqSARRecv ( 
	IN	IBT_DGRM_ELEMENT		*pDgrm,
	IN	uint64					TransactionID,
	IN	uint32					SegmentNum
	);

void
TimerCallbackRecv(
	IN	void					*TimerContext
	);

void
DestroyGsaTimer ( 
	IN GSA_SAR_CONTEXT			*pSarContext 
	);

FSTATUS
RemoveSARListEntry(
	IN	LIST_ITEM				*ListItem
	);

// ---------------------------------------------------------------------------
// Externally visible common functions used by GSA

// Destroy Contexts for any SAR in progress.
// designed to only be called in GsaUnload
// since RemoveDevice already destroyed QPs, we will not end up with
// new work on the list
void
DestroySARContexts()
{
	GSA_SAR_CONTEXT*	pSarContext;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, DestroySARContexts );

	// destroy sends in progress
	SpinLockAcquire( &g_GsaGlobalInfo->SARSendListLock);
	while ( NULL != (pSarContext = (GSA_SAR_CONTEXT*)QListHead( &g_GsaGlobalInfo->SARSendList )))
	{
		// BUGBUG there are races in SARCloseSend since no locks are held
		// while it is called, context could go away or race with
		// a timer.  IB1.1 is our focus, so this will not be fixed

		// must release list lock so Context can remove itself from list
		SpinLockRelease( &g_GsaGlobalInfo->SARSendListLock);
		// call with list lock unheld
		SARCloseSend(pSarContext, FALSE);
		SpinLockAcquire( &g_GsaGlobalInfo->SARSendListLock);
	}
	SpinLockRelease( &g_GsaGlobalInfo->SARSendListLock);

	// destroy receives in progress
	SpinLockAcquire( &g_GsaGlobalInfo->SARRecvListLock);
	while ( NULL != (pSarContext = (GSA_SAR_CONTEXT*)QListHead( &g_GsaGlobalInfo->SARRecvList )))
	{
		// BUGBUG there are races in DestroyGsaTimer since no locks are held
		// while it is called, context could go away or race with
		// a timer.  IB1.1 is our focus, so this will not be fixed

		// must release list lock so Context can remove itself from list
		SpinLockRelease( &g_GsaGlobalInfo->SARRecvListLock);
		DgrmPoolPut( pSarContext->RecvHead );
		DestroyGsaTimer( pSarContext );
		SpinLockAcquire( &g_GsaGlobalInfo->SARRecvListLock);
	}
	SpinLockRelease( &g_GsaGlobalInfo->SARRecvListLock);
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
}

// Completion Handlers shared by RMPP Receiver and Sender

// called on GSI Receive Q completion callback when a packet is received which
// is destined to a GSI Client who has requested GSI to do the
// RMPP Segmentation and Assembly
// This can be called for RMPP Contexts which are Senders or Receivers
FSTATUS
ProcessSARRecvCompletion (
	IN	MAD						*pMad,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	)
{
	FSTATUS						status;
	IBT_DGRM_ELEMENT_PRIV		*pDgrmPrivate;
	MAD_SAR						*pMadSar = (MAD_SAR*)pMad;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, ProcessSARRecvCompletion );

	pDgrmPrivate			= (IBT_DGRM_ELEMENT_PRIV *)*ppDgrm;

	// Overload base if it is SAR data for Coalesce API
	pDgrmPrivate->Base++;

	BSWAP_SAR_HEADER( &pMadSar->Sar );
	_DBG_PRINT_SAR_HEADERS( (_DBG_LVL_CALLBACK + _DBG_LVL_SAR_INFO),
		"SAR Recv Info:\n", pMadSar, pDgrmPrivate->DgrmElement.RemoteLID);
	//
	// Check for KeepAlive packets - Receiver Context
	//
	if (pMadSar->Sar.FragmentFlag.F.KeepAlive)
	{
		_DBG_PRINT(_DBG_LVL_SAR_INFO, (
			"KeepAlive packet received. Restart timer\n"
			));

		if (TRUE == KeepAliveSARRecv( pMadSar, ppDgrm ))
		{
			DgrmPoolPut( *ppDgrm );
			status = FPENDING;		// Do not callback user
		}
		else
		{
			status = FCOMPLETED;	// Not GSI managed. pass it on
		}
	}
	else
	if (pMadSar->Sar.FragmentFlag.F.ResendReq)
	{
		// Resend Request - Sender Context
		_DBG_PRINT(_DBG_LVL_SAR_INFO, (
			"** Resend Request packet received at segment %d."
			"Reset segment number request **\n",
			pMadSar->Sar.SegmentNum
			));

		if (TRUE == ResendReqSARSend( pMadSar, ppDgrm ))
		{
			DgrmPoolPut( *ppDgrm );
			status = FPENDING;		// Do not callback user
		}
		else
		{
			status = FCOMPLETED;	// Not GSI managed. pass it on
		}
	}
	else
	if (pMadSar->Sar.FragmentFlag.F.Ack)
	{
		// Ack - Sender Context
		_DBG_PRINT(_DBG_LVL_SAR_INFO, (
			"** Ack packet received at segment %d\n",
			pMadSar->Sar.SegmentNum
			));

		if (TRUE == AckReqSARSend( pMadSar, ppDgrm ))
		{
			DgrmPoolPut( *ppDgrm );
			status = FPENDING;		// Do not callback user
		}
		else
		{
			status = FCOMPLETED;	// Not GSI managed. pass it on
		}
	}else
	if ( pMadSar->Sar.SegmentNum > 1 )	// Is it the first segment?
	{
		// Additional Data Block - Receiver Context
		//
		// No. It is a segment reassembly in progress.
		// Add this segment to the others already received.
		//
		status = SARRecv( ppDgrm );
	}
	else
	{
		//
		// First SAR Data Block - Receiver Context
		// Does the data contain segmented payload?
		//
		if ( 0 == pMadSar->Sar.SegmentNum )
		{
#if defined(DBG) || defined(IB_DEBUG)
			// validate fragment flag
			if ( pMadSar->Sar.FragmentFlag.F.FirstPkt || \
					pMadSar->Sar.FragmentFlag.F.LastPkt )
			{
				_DBG_ERROR((
					"Received BAD SAR Header:FragmentFlag!!! Forced receive\n"));
				//status = FREJECT;
			}
#endif

			// There's only a single segment to receive.  We're done.
			// payload cannot be validated, undefined for non-multipacket
			_DBG_PRINT(_DBG_LVL_SAR_INFO, (
				"Only one segment to receive\n"));
			status = FCOMPLETED;

			// Send Ack on this msg
			SendAckSARRecv(*ppDgrm);

		}
		else
		{
			// multi-packet SAR
			// Segment should be == 1. validate FragmentFlag

			// FirstPacket and LastPacket are set to 1 (Vol1.M.2)
			if (( 1 == pMadSar->Sar.FragmentFlag.F.FirstPkt ) && \
				( 1 == pMadSar->Sar.FragmentFlag.F.LastPkt ))
			{
				// single packet SAR.
				_DBG_PRINT(_DBG_LVL_SAR_INFO, (
					"Only one segment to receive\n"));
				// validate PayloadLen, SA can send bad packets
				if ( pMadSar->Sar.PayloadLen > MAD_SAR_DATA_SIZE)
				{
					_DBG_ERROR((
						"Received BAD SAR Header: Payload too large <%d>!!! <Discard>\n",
								pMadSar->Sar.PayloadLen));
					status = FREJECT;
				} else {
					status = FCOMPLETED;

					// Send Ack on this msg
					SendAckSARRecv(*ppDgrm);
				}
			}
			else
			//	FirstPacket should be one and Last packet should not be set
			if (( 1 != pMadSar->Sar.FragmentFlag.F.FirstPkt ) || \
				pMadSar->Sar.FragmentFlag.F.LastPkt )
			{
				_DBG_ERROR((
					"Received BAD SAR Header:FragmentFlag!!! <Discard>\n"));
				status = FREJECT;
			}
			else
			{
				// Validate payload length
				if ( pMadSar->Sar.PayloadLen > MAD_SAR_DATA_SIZE)
				{
					// Start the reassembly.
					status = CreateSARRecv( *ppDgrm );
				}
				else
				{
					// The size is too small.
					_DBG_ERROR(("Received BAD SAR Header: Payload too small <%d>!!! <Discard>\n",
								pMadSar->Sar.PayloadLen));
					status = FREJECT;
				}
			}		// FragmentFlag
		}			// segment == 1
	}				// segment > 1

	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return status;
}

// called on GSI Send Q completion callback when a packet is received which
// is destined to a GSI Client who has requested GSI to do the
// RMPP Segmentation and Assembly
// This can be called for RMPP Contexts which are Senders or Receivers
void
ProcessSARSendCompletion (
	IN	MAD						*pMad,
	IN	IBT_DGRM_ELEMENT		*pDgrm
	)
{
	FSTATUS						status;
	IBT_DGRM_ELEMENT_PRIV	*pDgrmPrivate;
	GSA_POST_SEND_LIST		*pPostList;
	MAD_SAR					*pMadSar = (MAD_SAR*)pMad;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, ProcessSARSendCompletion );

	pDgrmPrivate			= (IBT_DGRM_ELEMENT_PRIV *)pDgrm;
	pPostList		= pDgrmPrivate->Base;
	BSWAP_SAR_HEADER (&pMadSar->Sar);

	// Check for Ack msg sent on behalf of RMPP Receiver
	if( (pMadSar->Sar.FragmentFlag.F.Ack || \
		pMadSar->Sar.FragmentFlag.F.ResendReq) )
	{
#if defined(DBG) || defined(IB_DEBUG)
		_DBG_PRINT_SAR_HEADERS( (_DBG_LVL_CALLBACK + _DBG_LVL_SAR_INFO),
			"SAR Ack/ResendReq packet send completed\n",
			pMadSar, pDgrmPrivate->DgrmElement.RemoteLID);
#endif
		ASSERT ( pDgrmPrivate->AvHandle );
		status = PutAV(
						NULL,
						pDgrmPrivate->AvHandle );

		// Just return to pool
		status = DgrmPoolPut( pPostList->DgrmList );
		goto done;
	}

	// must have been a send completion on behalf of RMPP Sender
	// BUGBUG - race, what if Gsi client deregisters before send completion?
	ASSERT(((GSA_SERVICE_CLASS_INFO*)pDgrmPrivate->MemPoolHandle->ServiceHandle)->bSARRequired);
				
	// BUGBUG If the completion had an error, such as flush for
	// unloading module, we may attempt to send SAR here
	// we should handle this special case and return to
	// user callback with an error indication
	// maybe we are ok, as we will post sends and get failures
	// until SAR is completed.

	_DBG_PRINT_SAR_HEADERS( (_DBG_LVL_CALLBACK + _DBG_LVL_SAR_INFO),
		"SAR send completed:\n",
		pMadSar, pDgrmPrivate->DgrmElement.RemoteLID);

	SysCallbackQueue( 
		pDgrmPrivate->pSarContext->SysCBItem, 
		SARSendSegments,
		pDgrmPrivate->pSarContext, 
		FALSE );

done:
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
}

// ---------------------------------------------------------------------------
// RMPP Sender

//
// DoSARSend
//
//	Used by GsiPostSendDgrm to perform a send for a class which
//	has requested Segmentation And Reassembly to be performed for it
//	by GSI
//
//
// INPUTS:
//
//	ServiceClass	- The clients class info
//
//	PostList		- Post list info
//
//	
//
// OUTPUTS:
//
//	None
//
//
//
// RETURNS:
//
//	FCANCELED - Segmentation not required, GsiPostSendDgrm should just send
//			the single packet request
//	FSUCCESS - Segmentation and transmission process has begun
//	other	- unable to start transmission
//
FSTATUS
DoSARSend(
	IN	GSA_SERVICE_CLASS_INFO	*ServiceClass, 
	IN	GSA_POST_SEND_LIST		*PostList
	)
{
	FSTATUS						status		= FSUCCESS;

	uint32						sendSize;
	IBT_DGRM_ELEMENT_PRIV		*pDgrmPrivate;
	MAD_COMMON					*pMadHdr;
	MULTI_PKT_HDR				*pSARHdr;
	IBT_BUFFER					*pBuffer;
	uint32						dgrmMsgSize, dgrmCount;
	IB_HANDLE					qp1Handle, cqHandle;
	SPIN_LOCK					*qpLock;
	IB_L_KEY					memLKey;
	GSA_SAR_CONTEXT				*pSarContext;
	IB_WORK_REQ					*pWorkRequest;
	IB_ADDRESS_VECTOR			avInfo;
	
	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, DoSARSend);

	
	//
	// Get total send requests
	//
	
	pDgrmPrivate		= (IBT_DGRM_ELEMENT_PRIV *)PostList->DgrmList;
	pDgrmPrivate->Base	= PostList;		// link for receives
			
	pMadHdr				= 
		(MAD_COMMON *)GsiDgrmGetSendMad(&pDgrmPrivate->DgrmElement);

	pSARHdr				= 
		(MULTI_PKT_HDR *)((uintn)pMadHdr + sizeof(MAD_COMMON));

	sendSize			= 
		pSARHdr->PayloadLen + sizeof(MAD_COMMON) + sizeof(MULTI_PKT_HDR);

	
	//
	// Validate user buffer size with payload
	//
	pBuffer				= pDgrmPrivate->DgrmElement.Element.pBufferList;
	dgrmMsgSize			= 0;

	while ( NULL != pBuffer )
	{
		dgrmMsgSize		+= pBuffer->ByteCount;
		pBuffer			= pBuffer->pNextBuffer;
	}

	for ( ;; )
	{
		if ( sendSize > dgrmMsgSize )
		{
			_DBG_ERROR ((
				"User passed Payload that overruns buffer!!!\n"
				"\tMsg size.........:x%x\n"
				"\tBuffer size......:x%x\n",
				sendSize,
				dgrmMsgSize ));

			status = FINVALID_PARAMETER;
			break;
		}
		
		//
		// Calculate segments to send
		//
		dgrmCount = 1;

		if ( sendSize > sizeof(MAD) )
		{
			sendSize -= (sizeof(MAD_COMMON) + sizeof(MULTI_PKT_HDR));

			if (!( sendSize % MAD_SAR_DATA_SIZE))
				dgrmCount--;

			dgrmCount += ( sendSize / MAD_SAR_DATA_SIZE);
			
			_DBG_PRINT(_DBG_LVL_SAR_INFO,(
				"SAR segments = %d\n", 
				dgrmCount ));
		}
		else
		{
			_DBG_PRINT(_DBG_LVL_SAR_INFO,(
				"Only one SAR segment\n" ));
		}

		//
		// Init SAR Hdr
		//
		pSARHdr->SegmentNum						= 0;

		pSARHdr->FragmentFlag.AsReg8 = 0;
		pSARHdr->Window							= 0;
			
		
		//
		// If there are multiple SAR segments go here
		//
		if ( dgrmCount == 1 )
		{
			status								= FCANCELED;
			_DBG_PRINT(_DBG_LVL_SAR_INFO,(
				"No SAR required. Do a normal send\n" ));

			break;
		}

		// send one at a time 
		PostList->DgrmIn						= 1;
		PostList->SARRequest					= TRUE;

		//
		// Fill in Address Vector Info
		//
		avInfo.PortGUID			= pDgrmPrivate->DgrmElement.PortGuid;
		avInfo.DestLID			= pDgrmPrivate->DgrmElement.RemoteLID;
		avInfo.PathBits			= pDgrmPrivate->DgrmElement.PathBits;
		avInfo.ServiceLevel		= pDgrmPrivate->DgrmElement.ServiceLevel;
		avInfo.StaticRate		= pDgrmPrivate->DgrmElement.StaticRate;
		avInfo.GlobalRoute		= pDgrmPrivate->DgrmElement.GlobalRoute;

		if ( TRUE == avInfo.GlobalRoute )
		{
			/* structure assignment */
			avInfo.GlobalRouteInfo	= pDgrmPrivate->DgrmElement.GRHInfo;
		}

		//
		// Query transport specific info for all segments in one go
		//
		status = GetCaContextFromAvInfo(
			&avInfo,
			pDgrmPrivate->MemPoolHandle->MemList->CaMemIndex,
			&pDgrmPrivate->AvHandle,
			&qp1Handle,
			&cqHandle,
			&qpLock,
			&memLKey );

		if ( FSUCCESS != status )
		{
			_DBG_ERROR ((
				"Error in GetCaContextFromAvInfo()\n" ));
			break;
		}

		//
		// If this is a request, save the user's transaction id 
		// and use our own.  We will restore the user's transaction id
		// when the request completes.  If this is a response, we need to
		// respond with the given transaction id, so that the remote side
		// can match it with its original request.
		if (MAD_IS_REQUEST((MAD*)pMadHdr))
		{
			pMadHdr->TransactionID = (uint64)\
				(pMadHdr->TransactionID & 0xffffffffffffff00ll);
			pMadHdr->TransactionID = \
				pMadHdr->TransactionID | ServiceClass->ClientId;
		}

		pSarContext = (GSA_SAR_CONTEXT*)MemoryAllocateAndClear(sizeof(GSA_SAR_CONTEXT), FALSE, GSA_MEM_TAG);
		if ( NULL == pSarContext )
		{
			status								= FINSUFFICIENT_RESOURCES;
			_DBG_ERROR ((
					"Not enough memory for Sar Context info!\n" ));
			break;
		}		

		//
		// Init Lock and Timer
		//
		SpinLockInitState( &pSarContext->Lock );
		SpinLockInit( &pSarContext->Lock );

		TimerInitState(&pSarContext->TimerObj);
		if (TRUE != TimerInit( 
					&pSarContext->TimerObj, 
					TimerCallbackSend, 
					pSarContext ))
		{
			status								= FINSUFFICIENT_RESOURCES;
			_DBG_ERROR ((
					"TimerInit failed!\n" ));

			SpinLockDestroy( &pSarContext->Lock );
			MemoryDeallocate( pSarContext );
			break;
		}		

		//
		// setup other fields
		//
		pSarContext->TotalSegs						= dgrmCount;
		pSarContext->CurrSeg						= 0;

		pSarContext->TID							= pMadHdr->TransactionID;
		pSarContext->MgmtClass						= pMadHdr->MgmtClass;
		
		pSarContext->Qp1Handle						= qp1Handle;
		pSarContext->QpLock							= qpLock;
		pSarContext->PostList						= PostList;

		pBuffer									= pDgrmPrivate->DgrmElement.Element.pBufferList;

		pSarContext->TotalDataSize					= pSARHdr->PayloadLen;
		pSarContext->DataStart						= 
					(uintn)(pBuffer->pData) + \
					sizeof(MAD_COMMON) + \
					sizeof(MULTI_PKT_HDR);

		_DBG_PRINT (_DBG_LVL_SAR_INFO,(
					"SAR Payload Length = x%x bytes\n",pSarContext->TotalDataSize ));

		//
		// Format SAR header
		//
		pSARHdr->SegmentNum						= 0;	
		pSARHdr->FragmentFlag.AsReg8				= 0;
		pSARHdr->FragmentFlag.F.FirstPkt			= 1;

		//
		// Pre create work request since it is common for all segments
		//
		pWorkRequest							= &pSarContext->WorkRequest;

		ASSERT_VALID_WORKREQID((uintn)pDgrmPrivate);
		pWorkRequest->WorkReqId					= BUILD_SQ_WORKREQID((uintn)pDgrmPrivate);
		
		//
		// SAR will Always be 3: MAD_COMMON, SAR_HDR, DATA
		//
		pWorkRequest->DSListDepth				= 3;		
		
		pWorkRequest->DSList					= pDgrmPrivate->DsList;
		pWorkRequest->Req.SendUD.AVHandle		= pDgrmPrivate->AvHandle;

		pWorkRequest->Operation					= WROpSend;

		// zero out the options
  		pWorkRequest->Req.SendUD.Options.AsUINT16				= 0;

		pWorkRequest->Req.SendUD.Options.s.SignaledCompletion	= TRUE; 

		//pWorkRequest->Req.SendUD.Qkey			= pDgrmPrivate->DgrmElement.RemoteQKey;
		pWorkRequest->Req.SendUD.QPNumber		= pDgrmPrivate->DgrmElement.RemoteQP;
		pWorkRequest->Req.SendUD.Qkey			= QP1_WELL_KNOWN_Q_KEY;

#if defined(DBG) || defined(IB_DEBUG)

		if (!pWorkRequest->Req.SendUD.QPNumber) 
		{
			AtomicIncrementVoid( &ServiceClass->ErrorMsgs );
			
			if (!(AtomicRead(&ServiceClass->ErrorMsgs) % 20))
			{
				_DBG_ERROR((
					"Invalid QPNumber/Q_Key(substitution provided):\n"
					"\tClass...........:x%x\n"
					"\tResponder.......:%s\n"
					"\tTrapProcessor...:%s\n"
					"\tReportProcessor.:%s\n"
					"\tSARClient.......:%s\n"
					"\tQPNumber........:x%x\n"
					"\tQ_Key...........:x%x\n",
					ServiceClass->MgmtClass,
					GSI_IS_RESPONDER(ServiceClass->RegistrationFlags)
						? "TRUE":"FALSE",
					GSI_IS_TRAP_PROCESSOR(ServiceClass->RegistrationFlags)
						? "TRUE":"FALSE",
					GSI_IS_REPORT_PROCESSOR(ServiceClass->RegistrationFlags)
						? "TRUE":"FALSE",
					ServiceClass->bSARRequired ? "TRUE":"FALSE",
					pWorkRequest->Req.SendUD.QPNumber,
					pWorkRequest->Req.SendUD.Qkey
					));
			}

			pWorkRequest->Req.SendUD.Qkey		= QP1_WELL_KNOWN_Q_KEY;
			pWorkRequest->Req.SendUD.QPNumber	= 1;
		}

		if (pWorkRequest->Req.SendUD.QPNumber	!= 1)
		{
			_DBG_ERROR((
				"Send to RedirectedQP\n"
				"\tQPNumber.....:x%x\n",
				pWorkRequest->Req.SendUD.QPNumber
				));
		}

#endif			

		//
		// fill in scatter/gather DS list details
		//
		pWorkRequest->MessageLen				= 0;

		//
		// Fill in MAD Common info
		//
		pWorkRequest->DSList[0].Address			= (uintn)pBuffer->pData;
		pWorkRequest->DSList[0].Lkey			= memLKey;
		pWorkRequest->DSList[0].Length			= sizeof(MAD_COMMON);

		//
		// Fill in SAR Header info
		//
		pWorkRequest->DSList[1].Address			= \
			(uintn)(pBuffer->pData) + sizeof(MAD_COMMON);
		pWorkRequest->DSList[1].Lkey			= memLKey;
		pWorkRequest->DSList[1].Length			= sizeof(MULTI_PKT_HDR);

		//
		// Fill in Data info
		//
		pWorkRequest->DSList[2].Lkey			= memLKey;
		
		pSarContext->DataSeg					= &pWorkRequest->DSList[2];
	
		//
		// TBD: Add to master SAR send list
		//
		SpinLockAcquire( &g_GsaGlobalInfo->SARSendListLock );

		QListInsertHead(
			&g_GsaGlobalInfo->SARSendList, 
			&pSarContext->ListItem );

		SpinLockRelease( &g_GsaGlobalInfo->SARSendListLock );

		//
		// Add timer to Dgrm internal list
		//
		pDgrmPrivate->pSarContext					= pSarContext;

		pSarContext->SysCBItem						= SysCallbackGet( 
													IbtGlobal.DeviceObject );
		if ( NULL == pSarContext->SysCBItem )
		{
			status								= FINSUFFICIENT_RESOURCES;
			TimerDestroy( &pSarContext->TimerObj );
			SpinLockDestroy( &pSarContext->Lock );
			MemoryDeallocate ( pSarContext );
			_DBG_ERROR ((
				"SysCallback Create failed! Dropping SAR send req.\n" ));
			break;
		}
		
		SARPost(pSarContext);
		break;
	}			// for loop

    _DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return status;
}


FSTATUS
SARPost(
	IN	GSA_SAR_CONTEXT		*pSarContext
	)
{
	FSTATUS					status		=FSUCCESS;
	IBT_DGRM_ELEMENT_PRIV	*pDgrmPrivate;
	IB_LOCAL_DATASEGMENT	*pDsList;
	IB_WORK_REQ				*pWorkRequest;
	MULTI_PKT_HDR			*pSARHdr;
	uint32					bytesSent, bytesToSend;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, SARPost);


	SpinLockAcquire( &pSarContext->Lock );

	pDgrmPrivate			= 
		(IBT_DGRM_ELEMENT_PRIV *)pSarContext->PostList->DgrmList;

	pWorkRequest			= &pSarContext->WorkRequest;
	pSARHdr					= \
		(MULTI_PKT_HDR *)((uintn)pWorkRequest->DSList[1].Address);

	//
	// Calculate bytes to be sent
	//
	pDsList					= pSarContext->DataSeg;

	bytesSent				= pSarContext->CurrSeg * MAD_SAR_DATA_SIZE;
	bytesToSend				= pSarContext->TotalDataSize - bytesSent;

	pDsList->Address		= pSarContext->DataStart + bytesSent;
	pDsList->Length			= MAD_SAR_DATA_SIZE;

	pWorkRequest->MessageLen= pDsList->Length + \
		sizeof(MAD_COMMON) + sizeof(MULTI_PKT_HDR);


	//
	// Check to see if we have multiple segments
	//
	if ( 0 != pSarContext->CurrSeg )
	{
		pSARHdr->FragmentFlag.F.FirstPkt = 0;

		if ( pSarContext->CurrSeg+1 == pSarContext->TotalSegs )
			pSARHdr->FragmentFlag.F.LastPkt = 1;
	}
	else
	{
		//
		// First segment. Validate payload needs multi segments
		//
		pSARHdr->FragmentFlag.F.FirstPkt = 1;
	}

	AtomicIncrementVoid( &pSarContext->CurrSeg );	// advance to next level
	pSARHdr->SegmentNum = pSarContext->CurrSeg;

	_DBG_PRINT (_DBG_LVL_SAR_INFO,(
		"SAR Header Info:\n"
		"\tPayloadLen...:%d bytes\n"
		"\tSegment......:%d\n",
		pSARHdr->PayloadLen,
		pSARHdr->SegmentNum
		));
	_DBG_PRINT (_DBG_LVL_SAR_INFO,(
		"SAR Header Info (cont):\n"
		"\tFragmentFlag.:\n"
		"\t\tFirstPkt..:x%x\n"
		"\t\tLastPkt...:x%x\n"
		"\t\tKeepAlive.:x%x\n"
		"\t\tResendReq.:x%x\n"
		"\t\tAck.......:x%x\n",
		(uint8)(pSARHdr->FragmentFlag.F.FirstPkt),
		(uint8)(pSARHdr->FragmentFlag.F.LastPkt),
		(uint8)(pSARHdr->FragmentFlag.F.KeepAlive),
		(uint8)(pSARHdr->FragmentFlag.F.ResendReq),
		(uint8)(pSARHdr->FragmentFlag.F.Ack)						
		));
	
	AtomicWrite(&pSarContext->PostList->DgrmOut, 0);

	pSarContext->RetryCount			= 0;
	pSarContext->ResendSent			= FALSE;

	//
	// Byte order headers
	//
	BSWAP_MAD_HEADER ( (MAD*)((uintn)pWorkRequest->DSList[0].Address) );

	BSWAP_SAR_HEADER ( pSARHdr );


	//
	// Submit to verbs
	//
	SpinLockAcquire( pSarContext->QpLock );

	status = iba_post_send( pSarContext->Qp1Handle, pWorkRequest );

	SpinLockRelease( pSarContext->QpLock );

	SpinLockRelease( &pSarContext->Lock );

	_DBG_LEAVE_LVL (_DBG_LVL_SAR_INFO);
	return status;
}


boolean
ResendReqSARSend( 
	IN	MAD_SAR					*pMadSar,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	)
{
	IBT_DGRM_ELEMENT			*pDgrm;
	GSA_SAR_CONTEXT				*pSarContext= NULL;
	boolean						bStatus		= FALSE;

	LIST_ITEM					*pListFirst, *pListNext;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, ResendReqSARSend);


	for ( ;; )
	{
		pDgrm = *ppDgrm;

		_DBG_PRINT (_DBG_LVL_SAR_INFO,(
			"Need to find TID x%"PRIx64"\n", 
			pMadSar->common.TransactionID ));

		SpinLockAcquire( &g_GsaGlobalInfo->SARSendListLock );

		pListFirst = QListHead( &g_GsaGlobalInfo->SARSendList );

    	if ( pListFirst )
		{
			pSarContext=NULL;

			while ( pListFirst )
			{
				//
				// Dump debug info
				//
				_DBG_PRINT (_DBG_LVL_SAR_INFO,(
					"...TID %"PRIx64"\n", 
					((GSA_SAR_CONTEXT*)pListFirst)->TID ));

				//
				// IB defines a message as unique by the combination of
				//	+ IB_GID from the GRH ( if applicable) TBD
				//	+ SourceLID
				//	+ TransactionID
				//

				// NOTE:This check will only fail if 2 receives arrive
				//		from the same source on the same LID and the
				//		TransactionID is the same.
				//		Clients issuing multiple requests to a class
				//		manager at the same time must pass in unique TIDs
				
				if (( ((GSA_SAR_CONTEXT*)pListFirst)->TID == pMadSar->common.TransactionID ) &&
					( ((GSA_SAR_CONTEXT*)pListFirst)->MgmtClass == pMadSar->common.MgmtClass ) &&
					( ((GSA_SAR_CONTEXT *)pListFirst)->PostList->DgrmList->RemoteLID == pDgrm->RemoteLID ))
				{
					pSarContext = (GSA_SAR_CONTEXT *)pListFirst;

					SpinLockAcquire( &pSarContext->Lock );

					// Reset Segment number to request
					pSarContext->CurrSeg = pMadSar->Sar.SegmentNum-1;

					SpinLockRelease( &pSarContext->Lock );

					bStatus = TRUE;
					_DBG_PRINT (_DBG_LVL_SAR_INFO,(
						"Timer restarted\n" ));
					break;
				}

				//
				// Walk entire chain of devices
				//

				pListNext = QListNext(
					&g_GsaGlobalInfo->SARSendList, 
					pListFirst );

				pListFirst = pListNext;

			}	// while 1

			//
			// This is how we find the end of the list
			//
			_DBG_PRINT (_DBG_LVL_SAR_INFO,(
					"Reached end of List\n" ));
		}

    
		//
		// Release the lock that we acquired earlier 
		//
		SpinLockRelease(&g_GsaGlobalInfo->SARSendListLock);
		break;

	}		// for loop

	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return (bStatus);
}	


boolean
AckReqSARSend( 
	IN	MAD_SAR					*pMadSar,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	)
{
	IBT_DGRM_ELEMENT			*pDgrm;
	GSA_SAR_CONTEXT				*pSarContext= NULL;
	boolean						bStatus		= FALSE;

	LIST_ITEM					*pListFirst, *pListNext;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, AckReqSARSend);


	for ( ;; )
	{
		pDgrm = *ppDgrm;

		_DBG_PRINT (_DBG_LVL_SAR_INFO,(
			"Need to find TID x%"PRIx64"\n", 
			pMadSar->common.TransactionID ));

		SpinLockAcquire( &g_GsaGlobalInfo->SARSendListLock );

		pListFirst = QListHead( &g_GsaGlobalInfo->SARSendList );

    	if ( pListFirst )
		{
			pSarContext=NULL;

			while ( pListFirst )
			{
				//
				// Dump debug info
				//
				_DBG_PRINT (_DBG_LVL_SAR_INFO,(
					"...TID %"PRIx64"\n", 
					((GSA_SAR_CONTEXT *)pListFirst)->TID ));

				//
				// IB defines a message as unique by the combination of
				//	+ IB_GID from the GRH ( if applicable) TBD
				//	+ SourceLID
				//	+ TransactionID
				//

				// NOTE:This check will only fail if 2 receives arrive
				//		from the same source on the same LID and the
				//		TransactionID is the same.
				//		Clients issuing multiple requests to a class
				//		manager at the same time must pass in unique TIDs
				
				if (( ((GSA_SAR_CONTEXT *)pListFirst)->TID == pMadSar->common.TransactionID ) &&
					( ((GSA_SAR_CONTEXT *)pListFirst)->MgmtClass == pMadSar->common.MgmtClass ) &&
					( ((GSA_SAR_CONTEXT *)pListFirst)->PostList->DgrmList->RemoteLID == pDgrm->RemoteLID ))
				{
					pSarContext = (GSA_SAR_CONTEXT *)pListFirst;

					SpinLockAcquire( &pSarContext->Lock );

					// Close send timer
					if ( pSarContext->TotalSegs == pSarContext->CurrSeg )
					{
						SpinLockRelease( &pSarContext->Lock );
						SARCloseSend(pSarContext, TRUE);
					}
					else
					{
						// we never asked for this Ack, do nothing
						SpinLockRelease( &pSarContext->Lock );
					}

					bStatus = TRUE;
					_DBG_PRINT (_DBG_LVL_SAR_INFO,(
						"Timer killed with Ack\n" ));
					break;
				}

				//
				// Walk entire chain of devices
				//

				pListNext = QListNext(
					&g_GsaGlobalInfo->SARSendList, 
					pListFirst );

				pListFirst = pListNext;

			}	// while 1

			//
			// This is how we find the end of the list
			//
			_DBG_PRINT (_DBG_LVL_SAR_INFO,(
					"Reached end of List\n" ));
		}
		else
		{
			//
			// This list has already gone away. 
			// We return TRUE since SAR client.
			//
			bStatus = TRUE;	// TBD BUGBUG - in consistent with not on list case
			_DBG_PRINT (_DBG_LVL_SAR_INFO,(
					"Info not in list. Delete anyway since SAR client\n" ));
		}

    
		//
		// Release the lock that we acquired earlier 
		//
		SpinLockRelease(&g_GsaGlobalInfo->SARSendListLock);
		break;

	}		// for loop

	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return (bStatus);
}	

//
// Close SAR send created
//
//	+ Destoy retry timer
//	+ Destroy AV
//	+ Destroy Syscallback
//	+ Call user or dump Dgrm
//	+ Free timer pool
//
void
SARCloseSend(
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN	boolean					callbackUser
	)
{
	IBT_DGRM_ELEMENT_PRIV		*pDgrmPrivate;
	GSA_SERVICE_CLASS_INFO		*classInfo;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, SARCloseSend);

	pDgrmPrivate				= 
		(IBT_DGRM_ELEMENT_PRIV *)pSarContext->PostList->DgrmList;

	_DBG_PRINT (_DBG_LVL_SAR_INFO,
		("*** Killing Send retry timer ***\n"));
			
	TimerStop( &pSarContext->TimerObj );		// stop timer before destroy
	TimerDestroy( &pSarContext->TimerObj );	// Destroy stops timer too

	PutAV(
		NULL,
		pDgrmPrivate->AvHandle );

	//
	// Remove from global list
	//
	SpinLockAcquire( &g_GsaGlobalInfo->SARSendListLock );

    ASSERT (QListIsItemInList( 
		&g_GsaGlobalInfo->SARSendList, 
		&pSarContext->ListItem ));


	if ( !QListIsItemInList( 
		&g_GsaGlobalInfo->SARSendList,
		&pSarContext->ListItem ))
	{
		_DBG_ERROR(("Item not in Send List!!!\n"));
	}
	else
	{
		QListRemoveItem ( 
			&g_GsaGlobalInfo->SARSendList,
			&pSarContext->ListItem );
	}
	
	SpinLockRelease(&g_GsaGlobalInfo->SARSendListLock);

	//
	// CallUser
	//
	if (callbackUser)
	{
		classInfo = pDgrmPrivate->MemPoolHandle->ServiceHandle;
		if ( classInfo->SendCompletionCallback )
		{
			(classInfo->SendCompletionCallback)( classInfo->ServiceClassContext,
								&pDgrmPrivate->DgrmElement );
		} else {
			// return to pool
			_DBG_PRINT (_DBG_LVL_SAR_INFO, (
				"No Class Agent for this SAR msg!\n"));
			DgrmPoolPut( &pDgrmPrivate->DgrmElement );	
		}
	} else {
		DgrmPoolPut( &pDgrmPrivate->DgrmElement );	
	}

	SpinLockDestroy( &pSarContext->Lock );		// lock
	SysCallbackPut( pSarContext->SysCBItem );
	MemoryDeallocate ( pSarContext );

    _DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);

}	// SARCloseSend




//
// Callback invoked on SAR sends
//
//	+ There are no retires for send.
//	+ Completions for ISR callbacks trigger next send of segement.
//	+ Timer is fired after last send
//
void
SARSendSegments(
	IN	void				*Key,
	IN	void				*TimerContext
	)
{
	GSA_SAR_CONTEXT			*pSarContext;
	IBT_DGRM_ELEMENT_PRIV	*pDgrmPrivate;
			

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, SARSendSegments);

	pSarContext				= TimerContext;
	pDgrmPrivate			= 
		(IBT_DGRM_ELEMENT_PRIV *)pSarContext->PostList->DgrmList;

	//
	// Verify last completion status
	//
	if ( FSUCCESS != pDgrmPrivate->DgrmElement.Element.Status )
	{
		_DBG_PRINT (_DBG_LVL_SAR_INFO,
			("*** Failed SAR send! Killing retry timer ***\n"));
			
		SARCloseSend( pSarContext, TRUE );
	}
	else
	{
		//
		// Are we in the last segment?
		//
		if ( pSarContext->TotalSegs == pSarContext->CurrSeg )
		{
			_DBG_PRINT (_DBG_LVL_SAR_INFO,
				("*** Kicking off retry timer ***\n"));

			//
			// Fire timer for retires and deliveries
			//
			TimerStart( &pSarContext->TimerObj, GSA_SAR_TIME_SEND );
		}
		else
		{
			SARPost(pSarContext);
		}
	}

    _DBG_LEAVE_LVL (_DBG_LVL_SAR_INFO);

}	// SARSendSegments


//
// Timer callback invoked on SAR sends
//	
//	+ Here the retry counter will stop posts if there is no completion.
//	+ The timer waits for the Isr routine's callback to update the send
//	  completions
//
void
TimerCallbackSend(
	IN	void				*TimerContext
	)
{
	GSA_SAR_CONTEXT				*pSarContext;
		

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, TimerCallbackSend);

	pSarContext					= TimerContext;
	SARCloseSend( pSarContext, TRUE );

    _DBG_LEAVE_LVL (_DBG_LVL_SAR_INFO);
}



// ---------------------------------------------------------------------------
// RMPP Receiver

boolean
IsRmppMethod(
	IN MAD_SAR					*pMadSarHdr
	)
{
	switch (pMadSarHdr->common.mr.AsReg8)
	{
	default:
		return FALSE;
		break;		// do nothing

	case 0x92:		// SubnAdminGetTableResp()
	case 0x14:		// SubnAdminGetMulti()
	case 0x94:		// SubnAdminGetMultiResp()

		return TRUE;
		break;
	}
}

//
// Create SAR Timer for recv
//
FSTATUS
CreateSARRecv(
	IN	IBT_DGRM_ELEMENT		*pDgrm
	)
{
	FSTATUS						status;
	MAD_SAR						*pMadSARHdr;
	GSA_SAR_CONTEXT				*pSarContext;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, CreateSARRecv);

	pMadSARHdr = (MAD_SAR *)GsiDgrmGetRecvMad(pDgrm);

	for ( ;; )
	{
		//
		// Inspect SAR received block for sanity
		//
		if ( pMadSARHdr->Sar.PayloadLen > GSA_LARGEST_SAR_BLOCK )
		{
			_DBG_ERROR ((
				"SAR Header contains a very large Payload(%d bytes)!\n"
				"GSA Max payload setting (%d bytes) <SAR Discard>\n",
				pMadSARHdr->Sar.PayloadLen,
				GSA_LARGEST_SAR_BLOCK ));

			status				= FREJECT;		
			break;
		}
		
		if (( pMadSARHdr->Sar.PayloadLen >  MAD_SAR_DATA_SIZE) &&
			( pDgrm->TotalRecvSize < 
			(sizeof(IB_GRH) + sizeof(MAD_COMMON) + sizeof(MULTI_PKT_HDR)) ))
		{
			_DBG_ERROR ((
				"Inconsistent SAR Header! <SAR Discard>\n"
				"\tPayload in Hdr......:%d bytes\n"
				"\tPayload received....:%d bytes\n",
				pMadSARHdr->Sar.PayloadLen,
				pDgrm->TotalRecvSize - \
				(sizeof(IB_GRH) + sizeof(MAD_COMMON) + sizeof(MULTI_PKT_HDR)) ));

			status				= FREJECT;
			break;
		}
		

		//
		// Create Timer info
		//
		pSarContext = (GSA_SAR_CONTEXT*)MemoryAllocateAndClear(sizeof(GSA_SAR_CONTEXT), FALSE, GSA_MEM_TAG);
		if ( NULL == pSarContext )
		{
			_DBG_ERROR ((
				"Not enough memory for Timer info! <SAR Discard>\n"));
			status				= FREJECT;
			break;
		}


		//
		// Init Lock and Timer
		//
		TimerInitState(&pSarContext->TimerObj);
		if (TRUE != TimerInit( 	&pSarContext->TimerObj, 
								TimerCallbackRecv, 
								pSarContext ))
		{
			_DBG_ERROR ((
				"TimerInit failed! <SAR Discard>\n" ));
			MemoryDeallocate ( pSarContext );
			status				= FREJECT;
			break;
		}

	
		SpinLockInitState( &pSarContext->Lock );
		SpinLockInit( &pSarContext->Lock );

		//
		// setup other fields
		//
		pSarContext->TotalSegs	= (pMadSARHdr->Sar.PayloadLen +MAD_SAR_DATA_SIZE-1)/ MAD_SAR_DATA_SIZE;
		pSarContext->MgmtClass	= pMadSARHdr->common.MgmtClass;
		pSarContext->CurrSeg	= 1;	// already received one
		pSarContext->TID		= pMadSARHdr->common.TransactionID;
		pSarContext->RecvHead	= pSarContext->RecvTail = pDgrm;
		pSarContext->TotalDataSize	= pMadSARHdr->Sar.PayloadLen;
		pSarContext->Window		= pMadSARHdr->Sar.Window;

		if (pSarContext->Window)
		{
			if (pSarContext->Window <64)
				pSarContext->Window = 64;

			pSarContext->NextWindow	= pSarContext->Window;
			pSarContext->NextWindow--;
		}

		_DBG_PRINT (_DBG_LVL_SAR_INFO,(
			"SAR Payload Length = x%x bytes\n",
			pSarContext->TotalDataSize ));

		// remove clientid from TID
		if (MAD_IS_RESPONSE((MAD*)pMadSARHdr))
		{
			pMadSARHdr->common.TransactionID = (uint64)\
				(pMadSARHdr->common.TransactionID & 0xffffffffffffff00ll);
		}

		//
		// Add to Global SAR recv list
		//
		SpinLockAcquire( &g_GsaGlobalInfo->SARRecvListLock );

		QListInsertHead(
			&g_GsaGlobalInfo->SARRecvList, 
			&pSarContext->ListItem );

		SpinLockRelease( &g_GsaGlobalInfo->SARRecvListLock );

		//
		// Start the timer to alert at the specified time.
		//
		if( !TimerStart( &pSarContext->TimerObj, GSA_SAR_TIME_RECV ) )
		{
			_DBG_ERROR ((
				"Error in Timer Start! <SAR Discard>\n"));

			SpinLockDestroy( &pSarContext->Lock );
			TimerDestroy( &pSarContext->TimerObj );

			//
			// Eject from Global SAR Q
			//
			RemoveSARListEntry( &pSarContext->ListItem );
			MemoryDeallocate ( pSarContext );
			status				= FREJECT;		
			break;
		}
	
		status					= FPENDING;
		break;
	}	// for loop

	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return status;
}


FSTATUS
SARRecv(
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	)
{
	IBT_DGRM_ELEMENT			*pDgrm;
	FSTATUS						status;
	GSA_SAR_CONTEXT				*pSarContext= NULL;

	MAD_SAR						*pMadSARHdr;
	LIST_ITEM					*pListFirst, *pListNext;
	boolean						bAckSent	= FALSE;
#ifdef TEST_SAR_RECV_DISCARD
	static int discard_counter = 0;
#endif

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, SARRecv);


	for ( ;; )
	{
		pDgrm = *ppDgrm;
		pMadSARHdr = (MAD_SAR *)GsiDgrmGetRecvMad(pDgrm);

		_DBG_PRINT (_DBG_LVL_SAR_INFO,(
			"Need to find TID x%"PRIx64"\n", 
			pMadSARHdr->common.TransactionID ));

		SpinLockAcquire( &g_GsaGlobalInfo->SARRecvListLock );

		pListFirst = QListHead( &g_GsaGlobalInfo->SARRecvList );

    	if ( !pListFirst )
		{
			//
			// the list is empty
			//
#ifdef TEST_SAR_RECV_DISCARD
			_DBG_ERROR(("Empty Global List"));
#endif
			_DBG_WARN((
				"Unexpected SAR Packet: No Entries in Global List! <Discarding>\n"
				"\tClass........:x%x\n"
				"\tAttributeID..:x%x\n"
				"\tTID..........:x%"PRIx64"\n"
				"\tSegment......:%d\n"
				"\tPayloadLen...:%d bytes\n",
				(uint8)pMadSARHdr->common.MgmtClass, 
				pMadSARHdr->common.AttributeID,
				pMadSARHdr->common.TransactionID,
				pMadSARHdr->Sar.SegmentNum,
				pMadSARHdr->Sar.PayloadLen
				));
			_DBG_WARN((
				"Unexpected SAR Packet: No Entries in Global List! (cont)\n"
				"\tFragmentFlag.:\n"
				"\t\tFirstPkt..:x%x\n"
				"\t\tLastPkt...:x%x\n"
				"\t\tKeepAlive.:x%x\n"
				"\t\tResendReq.:x%x\n"
				"\t\tAck.......:x%x\n",
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.FirstPkt),
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.LastPkt),
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.KeepAlive),
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.ResendReq),
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.Ack)						
				));
		}
		else
		{
			pSarContext=NULL;

			while ( pListFirst )
			{
				//
				// Dump debug info
				//
				_DBG_PRINT (_DBG_LVL_SAR_INFO,(
					"...TID %"PRIx64"\n", 
					((GSA_SAR_CONTEXT *)pListFirst)->TID ));

				//
				// IB defines a message as unique by the combination of
				//	+ IB_GID from the GRH ( if applicable) TBD
				//	+ SourceLID
				//	+ TransactionID
				//

				// NOTE:This check will only fail if 2 receives arrive
				//		from the same source on the same LID and the
				//		TransactionID is the same.
				//		Clients issuing multiple requests to a class
				//		manager at the same time must pass in unique TIDs
				
				if (( ((GSA_SAR_CONTEXT *)pListFirst)->TID == pMadSARHdr->common.TransactionID ) &&
					( ((GSA_SAR_CONTEXT *)pListFirst)->MgmtClass == pMadSARHdr->common.MgmtClass ) &&
					( ((GSA_SAR_CONTEXT *)pListFirst)->RecvHead->RemoteLID == pDgrm->RemoteLID ))
				{
					pSarContext = (GSA_SAR_CONTEXT *)pListFirst;
					break;
				}

				//
				// Walk entire chain of devices
				//

				pListNext = QListNext(
					&g_GsaGlobalInfo->SARRecvList, 
					pListFirst );

				pListFirst = pListNext;

			}	// while 1

			//
			// This is how we find the end of the list
			//
			_DBG_PRINT (_DBG_LVL_SAR_INFO,(
					"Reached end of List\n" ));
			if (NULL == pSarContext)
			{
#ifdef TEST_SAR_RECV_DISCARD
				_DBG_ERROR(("No recv Timer"));
#endif
				_DBG_WARN ((
					"Unexpected SAR Packet: No Recv Timer Queued in list! <Discarding>\n"
					"\tClass........:x%x\n"
					"\tAttributeID..:x%x\n"
					"\tTID..........:x%"PRIx64"\n"
					"\tSegment......:%d\n"
					"\tPayloadLen...:%d bytes\n",
					(uint8)pMadSARHdr->common.MgmtClass, 
					pMadSARHdr->common.AttributeID,
					pMadSARHdr->common.TransactionID,
					pMadSARHdr->Sar.SegmentNum,
					pMadSARHdr->Sar.PayloadLen
					));
				_DBG_WARN ((
					"Unexpected SAR Packet: (cont)\n"
					"\tFragmentFlag.:\n"
					"\t\tFirstPkt..:x%x\n"
					"\t\tLastPkt...:x%x\n"
					"\t\tKeepAlive.:x%x\n"
					"\t\tResendReq.:x%x\n"
					"\t\tAck.......:x%x\n",
					(uint8)(pMadSARHdr->Sar.FragmentFlag.F.FirstPkt),
					(uint8)(pMadSARHdr->Sar.FragmentFlag.F.LastPkt),
					(uint8)(pMadSARHdr->Sar.FragmentFlag.F.KeepAlive),
					(uint8)(pMadSARHdr->Sar.FragmentFlag.F.ResendReq),
					(uint8)(pMadSARHdr->Sar.FragmentFlag.F.Ack)						
					));
			}
		}

		//
		// Release the lock that we acquired earlier 
		//
		SpinLockRelease(&g_GsaGlobalInfo->SARRecvListLock);

#ifdef TEST_SAR_RECV_DISCARD
		// discard occasional SAR packet received, will stress retry mechanism
		if (NULL != pSarContext && ++discard_counter % TEST_SAR_RECV_DISCARD == 1)
		{
			_DBG_ERROR(("Test Discarding x%"PRIx64":%d\n",
				pMadSARHdr->common.TransactionID,
				pMadSARHdr->Sar.SegmentNum));
			_DBG_WARN ((
				"Test Discarding:\n"
				"\tClass........:x%x\n"
				"\tAttributeID..:x%x\n"
				"\tTID..........:x%"PRIx64"\n"
				"\tSegment......:%d\n"
				"\tPayloadLen...:%d bytes\n",
				(uint8)pMadSARHdr->common.MgmtClass, 
				pMadSARHdr->common.AttributeID,
				pMadSARHdr->common.TransactionID,
				pMadSARHdr->Sar.SegmentNum,
				pMadSARHdr->Sar.PayloadLen
				));
			_DBG_WARN ((
				"Test Discarding (cont):\n"
				"\tFragmentFlag.:\n"
				"\t\tFirstPkt..:x%x\n"
				"\t\tLastPkt...:x%x\n"
				"\t\tKeepAlive.:x%x\n"
				"\t\tResendReq.:x%x\n"
				"\t\tAck.......:x%x\n",
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.FirstPkt),
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.LastPkt),
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.KeepAlive),
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.ResendReq),
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.Ack)						
				));
			pSarContext= NULL;
		}
#endif
		if ( NULL == pSarContext )
		{
			AtomicIncrementVoid( &g_GsaGlobalInfo->SARRecvDropped );
			status = FREJECT;

			_DBG_PRINT(_DBG_LVL_SAR_INFO, (
					"*** SAR receive drop count (%d) msgs ***\n",
					g_GsaGlobalInfo->SARRecvDropped));
			break;
		}

		TimerStop( &pSarContext->TimerObj );
		// This is a hack.  I suspect there are some concurrency problems
		// in the existing algorithms.  This ensures timer callback is not
		// running while we do the code below.
		// There is a small race if timer is in its 1st few lines of callback
		// however that same race exists in the cases where we destroy the
		// timer below.  Given 1+ second timeout, race is minimal
		SpinLockAcquire(&pSarContext->Lock);
		SpinLockRelease(&pSarContext->Lock);

		//
		// Check Window to send Ack's
		//
		if (pSarContext->Window)
		{
			pSarContext->NextWindow--;
			if (!pSarContext->NextWindow)
			{
				// Send Ack
				SendAckSARRecv(pDgrm);

				// Only Ack once even if last msg
				bAckSent = TRUE;		

				// Reset it for next Window
				pSarContext->NextWindow = pSarContext->Window;
			}
		}

		//
		// Is it the next segment in list?
		//
		if ( pMadSARHdr->Sar.SegmentNum == (pSarContext->CurrSeg + 1) )
		{
#ifdef TEST_SAR_RECV_DISCARD
			_DBG_ERROR(("Got expected segment x%"PRIx64":%d\n",
				pMadSARHdr->common.TransactionID,
				pMadSARHdr->Sar.SegmentNum));
#endif
			// We have received the next expected segment.
			// Chain receives and update counters
			pSarContext->RecvTail->Element.pNextElement = (IBT_ELEMENT *)pDgrm;
			pSarContext->RecvTail = pDgrm;
			pSarContext->RecvHead->TotalRecvSize += pDgrm->Element.RecvByteCount;

			pSarContext->RetryCount = 0;
			pSarContext->ResendSent = FALSE;
			AtomicIncrementVoid( &pSarContext->CurrSeg );

			// See if the reassembly has completed.
			if (( pSarContext->CurrSeg == pSarContext->TotalSegs ) && \
				(!pMadSARHdr->Sar.FragmentFlag.F.LastPkt))
			{
				_DBG_PRINT(_DBG_LVL_SAR_INFO, (
					"Variable payload length being received.\n"
					"\tTotal Segs.......:%d\n"
					"\tpayload Length...:%d bytes\n"
					"\tWindow...........:%d\n",
					pSarContext->TotalSegs,
					pSarContext->TotalDataSize,
					pSarContext->Window ));
			}

			// account for variable payloads
			if (pMadSARHdr->Sar.FragmentFlag.F.LastPkt)	
			{
				// first defines payload len for SubnAdm(Config) and response
				uint32 PayloadLen = ((MAD_SAR *)GsiDgrmGetRecvMad(pSarContext->RecvHead))->Sar.PayloadLen;

				// Send last Ack
				if (TRUE != bAckSent)
					SendAckSARRecv(pDgrm);

				// for Variable length SARs, the PayloadLen in the Last
				// fragment takes precidence over the PayloadLen in the first
				if (pMadSARHdr->common.mr.AsReg8 == SUBN_ADM_GETTABLE_RESP
					|| pMadSARHdr->common.mr.AsReg8 == SUBN_ADM_GETBULK_RESP)
				{
					// this is number of bytes in this fragment
					// total payload is a function of number of segments
					PayloadLen = ((MAD_SAR *)GsiDgrmGetRecvMad(pSarContext->RecvHead))->Sar.PayloadLen =
						pMadSARHdr->Sar.PayloadLen
						+ (pMadSARHdr->Sar.SegmentNum-1) * MAD_SAR_DATA_SIZE;
				}

				//validate Number of Segments against PayLoadLen
				if ( pMadSARHdr->Sar.SegmentNum !=
						(PayloadLen +MAD_SAR_DATA_SIZE-1)/ MAD_SAR_DATA_SIZE)
				{
					// since we added to RecvHead, caller shouldn't free
					status = FPENDING;
					// HACK - just discard bad response caller will timeout
					_DBG_ERROR((
						"SAR with Incorrect payload length received! <Discard>.\n"
						"\tTotal Segs.......:%d\n"
						"\tpayload Length...:%d bytes\n"
						"\tExpected Segs....:%d\n",
						pMadSARHdr->Sar.SegmentNum,
						PayloadLen,
						(PayloadLen +MAD_SAR_DATA_SIZE-1)/ MAD_SAR_DATA_SIZE
						));
					// Return the dropped datagram segments to the datagram pool.
					DgrmPoolPut( pSarContext->RecvHead );
					AtomicIncrementVoid( &g_GsaGlobalInfo->SARRecvDropped );

					_DBG_PRINT (_DBG_LVL_SAR_INFO, (
						"*** SAR receive drop count (%d) msgs ***\n",
						g_GsaGlobalInfo->SARRecvDropped ));

					DestroyGsaTimer( pSarContext );
				} else {

					// The reassembly is complete.
					// Invoke the user callback for the completed datagram.
					status = FCOMPLETED;

					// remove clientid from TID
					if( MAD_IS_RESPONSE((MAD*)pMadSARHdr))
					{
						pMadSARHdr->common.TransactionID = (uint64)\
							(pMadSARHdr->common.TransactionID & 0xffffffffffffff00ll);
					}

					// Return all segments to the user or back to the 
					// datagram pool.
					*ppDgrm = pSarContext->RecvHead;
					DestroyGsaTimer( pSarContext );
				}
			}
			else
			{
				// We are still missing segments.  Wait until we receive
				// them all.
				status = FPENDING;

				// remove clientid from TID
				if( MAD_IS_RESPONSE((MAD*)pMadSARHdr))
				{
					pMadSARHdr->common.TransactionID = (uint64)\
						(pMadSARHdr->common.TransactionID & 0xffffffffffffff00ll);
				}

				// restart timer
				TimerStart( &pSarContext->TimerObj, GSA_SAR_TIME_RECV );
			}
		}
		else
		{
#ifdef TEST_SAR_RECV_DISCARD
			_DBG_ERROR(("Segment Mismatch x%"PRIx64":%d\n",
				pMadSARHdr->common.TransactionID,
				pMadSARHdr->Sar.SegmentNum));
#endif
			_DBG_WARN ((
				"SAR Packet: Segment number mismatch! <Discarding>\n"
				"\tClass........:x%x\n"
				"\tAttributeID..:x%x\n"
				"\tTID..........:x%"PRIx64"\n"
				"\tSegment......:%d\n"
				"\tPayloadLen...:%d bytes\n",
				(uint8)pMadSARHdr->common.MgmtClass, 
				pMadSARHdr->common.AttributeID,
				pMadSARHdr->common.TransactionID,
				pMadSARHdr->Sar.SegmentNum,
				pMadSARHdr->Sar.PayloadLen
				));

			_DBG_WARN ((
				"SAR Packet: Segment number mismatch! (cont)\n"
				"\tFragmentFlag.:\n"
				"\t\tFirstPkt..:x%x\n"
				"\t\tLastPkt...:x%x\n"
				"\t\tKeepAlive.:x%x\n"
				"\t\tResendReq.:x%x\n"
				"\t\tAck.......:x%x\n",
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.FirstPkt),
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.LastPkt),
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.KeepAlive),
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.ResendReq),
				(uint8)(pMadSARHdr->Sar.FragmentFlag.F.Ack)						
				));

			if ( pMadSARHdr->Sar.SegmentNum > (pSarContext->CurrSeg + 1) )
			{
				//
				// Request resend seg from here
				//
				// NOTE: We continuosly send Resend requests for every bad 
				// packet since the previous resends may have been dropped.
				//
				if (! pSarContext->ResendSent)
				{
					pSarContext->ResendSent = TRUE;
					pSarContext->RetryCount++;
					ResendReqSARRecv(pDgrm, pSarContext->TID, pSarContext->CurrSeg+1);
				}
				// else timer will perform resend retries
			}
			else
			{
				_DBG_WARN (( "SAR: Old packet in transaction!!! <Discarding>\n"));
			}

			// restart timer
			TimerStart( &pSarContext->TimerObj, GSA_SAR_TIME_RECV );

			status = FREJECT;
		}
		break;

	}		// for loop

	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return status;
}	


boolean
KeepAliveSARRecv( 
	IN	MAD_SAR					*pMadSar,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	)
{
	IBT_DGRM_ELEMENT			*pDgrm;
	GSA_SAR_CONTEXT				*pSarContext= NULL;
	boolean						bStatus		= FALSE;

	LIST_ITEM					*pListFirst, *pListNext;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, KeepAliveSARRecv);


	for ( ;; )
	{
		pDgrm = *ppDgrm;

		_DBG_PRINT (_DBG_LVL_SAR_INFO,(
			"Need to find TID x%"PRIx64"\n", 
			pMadSar->common.TransactionID ));

		SpinLockAcquire( &g_GsaGlobalInfo->SARRecvListLock );

		pListFirst = QListHead( &g_GsaGlobalInfo->SARRecvList );

    	if ( pListFirst )
		{
			pSarContext=NULL;

			while ( pListFirst )
			{
				//
				// Dump debug info
				//
				_DBG_PRINT (_DBG_LVL_SAR_INFO,(
					"...TID %"PRIx64"\n", 
					((GSA_SAR_CONTEXT *)pListFirst)->TID ));

				//
				// IB defines a message as unique by the combination of
				//	+ IB_GID from the GRH ( if applicable) TBD
				//	+ SourceLID
				//	+ TransactionID
				//

				// NOTE:This check will only fail if 2 receives arrive
				//		from the same source on the same LID and the
				//		TransactionID is the same.
				//		Clients issuing multiple requests to a class
				//		manager at the same time must pass in unique TIDs
				
				if (( ((GSA_SAR_CONTEXT *)pListFirst)->TID == pMadSar->common.TransactionID ) &&
					( ((GSA_SAR_CONTEXT *)pListFirst)->MgmtClass == pMadSar->common.MgmtClass ) &&
					( ((GSA_SAR_CONTEXT *)pListFirst)->RecvHead->RemoteLID == pDgrm->RemoteLID ))
				{
					pSarContext = (GSA_SAR_CONTEXT *)pListFirst;
					TimerStop( &pSarContext->TimerObj );
					TimerStart( &pSarContext->TimerObj, GSA_SAR_TIME_RECV );
					bStatus = TRUE;

					_DBG_PRINT (_DBG_LVL_SAR_INFO,(
						"Timer restarted\n" ));
					break;
				}

				//
				// Walk entire chain of devices
				//

				pListNext = QListNext(
					&g_GsaGlobalInfo->SARRecvList, 
					pListFirst );

				pListFirst = pListNext;

			}	// while 1

			//
			// This is how we find the end of the list
			//
			_DBG_PRINT (_DBG_LVL_SAR_INFO,(
					"Reached end of List\n" ));
		}

    
		//
		// Release the lock that we acquired earlier 
		//
		SpinLockRelease(&g_GsaGlobalInfo->SARRecvListLock);
		break;

	}		// for loop

	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return (bStatus);
}	


// uses pDgrm as a template for building a SAR Ack or resend request
// SAR Flags, SegmentNum and TID are from arguments
// rest is from pDgrm
// R bit is inverted
// pDgrm is unaffected
void
FragmentOptionsSARRecv (
	IN	IBT_DGRM_ELEMENT		*pDgrm,
	IN	FRAGMENT_FLAG			FragmentFlag,	// Ack or Resend?				
	IN	uint64					TransactionID,
	IN	uint32					SegmentNum
	)
{
	FSTATUS						status		= FSUCCESS;
	uint32						i;
	IBT_DGRM_ELEMENT			*pDgrmList;
	IB_WORK_REQ					workRequest;
	
	IB_HANDLE					qp1Handle = NULL, avHandle;
	SPIN_LOCK					*qpLock = NULL;
	IB_HANDLE					cq1Handle = NULL;

	GSA_POST_SEND_LIST			*pPostList;
	IBT_DGRM_ELEMENT_PRIV		*pDgrmPrivate;
	IB_LOCAL_DATASEGMENT		*pDsList;
	MAD_SAR						*pMadSarHdr;
	ADDRESS_VECTOR				avInfo;
	IB_L_KEY					LKey;
	

	_DBG_ENTER_LVL(_DBG_LVL_SAR_INFO, FragmentOptionsSARRecv);

	i = 1;
	status = DgrmPoolGet( g_GsaGlobalInfo->DgrmPoolRecvQ, &i, &pDgrmList );
	if (FSUCCESS != status || i != 1)
	{
		_DBG_ERROR(("No more elements in Pool!!!\n"));
		goto done;
	}
	ASSERT(NULL != pDgrmList);

	//
	// Copy Data for Ack/ResendReq packet and set SAR flags
	//
	pMadSarHdr = (MAD_SAR*)GsiDgrmGetSendMad(pDgrmList);

	MemoryCopy( pMadSarHdr, GsiDgrmGetRecvMad(pDgrm), sizeof(MAD) );

	// Invert R bit
	pMadSarHdr->common.mr.s.R = ! pMadSarHdr->common.mr.s.R;
	pMadSarHdr->common.TransactionID = TransactionID;
			
	// For Ack, First/Last/Resend should not be set
	pMadSarHdr->Sar.FragmentFlag.AsReg8 = 0;
	if (FragmentFlag.F.Ack)
		pMadSarHdr->Sar.FragmentFlag.F.Ack = 1;	
	else if (FragmentFlag.F.ResendReq)
		pMadSarHdr->Sar.FragmentFlag.F.ResendReq = 1;	
	pMadSarHdr->Sar.SegmentNum = SegmentNum;
	pMadSarHdr->Sar.PayloadLen = 0;

	// Byte order headers
	BSWAP_MAD_HEADER ((MAD*)&pMadSarHdr->common);
	BSWAP_SAR_HEADER (&pMadSarHdr->Sar);

	// Set remote info
	// incomplete and not needed since we PostSend directly
	//pDgrmList->PortGuid		= pDgrm->PortGuid;
	//pDgrmList->RemoteLID	= pDgrm->RemoteLID;
	//pDgrmList->RemoteQKey	= pDgrm->RemoteQKey;
	//pDgrmList->RemoteQP		= pDgrm->RemoteQP;

	//
	// Create Av Info
	//
	avInfo.PortGUID				= pDgrm->PortGuid;		
	avInfo.DestLID				= pDgrm->RemoteLID;		
	avInfo.PathBits				= pDgrm->PathBits;		
	avInfo.ServiceLevel			= pDgrm->ServiceLevel;
	avInfo.StaticRate			= pDgrm->StaticRate;		
	avInfo.GlobalRoute			= pDgrm->GlobalRoute;
	
	if ( TRUE == avInfo.GlobalRoute )
	{
		avInfo.GlobalRouteInfo.DestGID		= GsiDgrmGetRecvGrh(pDgrm)->SGID;
		avInfo.GlobalRouteInfo.FlowLabel	= pDgrm->GRHInfo.FlowLabel;
		avInfo.GlobalRouteInfo.SrcGIDIndex	= pDgrm->GRHInfo.SrcGIDIndex;	
		avInfo.GlobalRouteInfo.HopLimit		= pDgrm->GRHInfo.HopLimit;
		avInfo.GlobalRouteInfo.TrafficClass	= pDgrm->GRHInfo.TrafficClass;
	}
	
	//
	// post to qp
	// 
	pDgrmPrivate	= (IBT_DGRM_ELEMENT_PRIV *)pDgrmList;
	status = GetCaContextFromPortGuid(&avInfo,
					pDgrmPrivate->MemPoolHandle->MemList->CaMemIndex,
					&avHandle, &qp1Handle, &cq1Handle, &qpLock, &LKey);
	if (FSUCCESS != status)
	{
		_DBG_PRINT( _DBG_LVL_MGMT,(\
				"An active Port could not be found OR"
				"Qp in Error on Port(x%"PRIx64")!\n", pDgrm->PortGuid));

		DgrmPoolPut( pDgrmList );	
	} else {

		//
		// fill in work request details
		//

		//
		// generate a PostSendList from the first datagram in post
		//
		pPostList					= &pDgrmPrivate->PostSendList;

		//
		// Fill in list details
		//

		pPostList->DgrmIn = 0;
		AtomicWrite(&pPostList->DgrmOut, 0);
		pPostList->DgrmList			= pDgrmList;
		pPostList->SARRequest		= TRUE;

		pDgrmPrivate->Base			= pPostList;// link for receives
			
		ASSERT_VALID_WORKREQID((uintn)pDgrmPrivate);
		workRequest->WorkReqId		= BUILD_SQ_WORKREQID((uintn)pDgrmPrivate);
		workRequest.DSListDepth		= \
				pDgrmPrivate->MemPoolHandle->BuffersPerElement;

		workRequest.DSList = pDsList = pDgrmPrivate->DsList;

		workRequest.Operation = WROpSend;

		// zero out the options
  		workRequest.Req.SendUD.Options.AsUINT16 = 0;
		workRequest.Req.SendUD.Options.s.SignaledCompletion = TRUE; 

		//
		// Fill in user settings
		//
		workRequest.Req.SendUD.QPNumber = pDgrm->RemoteQP;
		workRequest.Req.SendUD.Qkey		= pDgrm->RemoteQKey;

		ASSERT (pDgrm->RemoteQP);
			
		pDgrmPrivate->AvHandle = workRequest.Req.SendUD.AVHandle = avHandle;

		//
		// fill in scatter/gather Ds list details
		//
		workRequest.DSListDepth			= 1;

		//
		// Fill in MAD Common info
		//
		pDsList[0].Address				= (uintn)pMadSarHdr;
		pDsList[0].Lkey					= LKey;
		pDsList[0].Length				= sizeof(MAD);
	
		workRequest.MessageLen			= sizeof(MAD);

			
		//
		// Submit to verbs
		//
		SpinLockAcquire( qpLock );
		status = iba_post_send( qp1Handle, &workRequest );
		SpinLockRelease( qpLock );
	}

done:
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
}


void
SendAckSARRecv ( 
	IN	IBT_DGRM_ELEMENT		*pDgrm
	)
{
	FRAGMENT_FLAG				fragFlags;
	MAD_SAR						*pMadSarHdr;


	_DBG_ENTER_LVL(_DBG_LVL_SAR_INFO, SendAckSARRecv);

	pMadSarHdr = (MAD_SAR*)GsiDgrmGetRecvMad(pDgrm);

	if (TRUE == IsRmppMethod(pMadSarHdr))
	{
		_DBG_PRINT (_DBG_LVL_SAR_INFO,(
			"SAR Packet: Sending Ack\n"
			"\tClass........:x%x\n"
			"\tAttributeID..:x%x\n"
			"\tTID..........:x%"PRIx64"\n"
			"\tSegment......:%d\n"
			"\tWindow.......:%d\n",
			(uint8)pMadSarHdr->common.MgmtClass, 
			pMadSarHdr->common.AttributeID,
			pMadSarHdr->common.TransactionID,
			pMadSarHdr->Sar.SegmentNum,
			pMadSarHdr->Sar.Window
			));

		fragFlags.AsReg8 = 0;
		fragFlags.F.Ack = 1;
		
		FragmentOptionsSARRecv (
			pDgrm,
			fragFlags,
			pMadSarHdr->common.TransactionID,
			pMadSarHdr->Sar.SegmentNum);
	}

	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
}

void
ResendReqSARRecv ( 
	IN	IBT_DGRM_ELEMENT		*pDgrm,
	IN	uint64					TransactionID,
	IN	uint32					SegmentNum
	)
{
	FRAGMENT_FLAG				fragFlags;
	MAD_SAR						*pMadSARHdr;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, ResendReqSARRecv );

	pMadSARHdr = (MAD_SAR *)GsiDgrmGetRecvMad(pDgrm);
#ifdef TEST_SAR_RECV_DISCARD
	_DBG_ERROR(("Requesting Resend of x%"PRIx64":%d\n",
				TransactionID,
				SegmentNum));
#endif
	_DBG_WARN ((
		"SAR Packet: Requesting Resend\n"
		"\tClass........:x%x\n"
		"\tAttributeID..:x%x\n"
		"\tTID..........:x%"PRIx64"\n"
		"\tSegment......:%d\n",
		(uint8)pMadSARHdr->common.MgmtClass, 
		pMadSARHdr->common.AttributeID,
		TransactionID,
		SegmentNum
		));

	fragFlags.AsReg8 = 0;
	fragFlags.F.ResendReq = 1;

	FragmentOptionsSARRecv (
		pDgrm,
		fragFlags,
		TransactionID,
		SegmentNum);

	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
}




//
// Remove from Global SAR recv Q
//
FSTATUS
RemoveSARListEntry(
	IN	LIST_ITEM					*ListItem
	)
{
    FSTATUS							status = FSUCCESS;
	

	_DBG_ENTER_LVL(_DBG_LVL_SAR_INFO, RemoveSARListEntry);

	SpinLockAcquire( &g_GsaGlobalInfo->SARRecvListLock );

    ASSERT (QListIsItemInList( 
		&g_GsaGlobalInfo->SARRecvList, 
		ListItem ));


	if ( !QListIsItemInList( 
		&g_GsaGlobalInfo->SARRecvList,
		ListItem ))
	{
		status						= FINVALID_PARAMETER;
		_DBG_ERROR(("Item not in List!!!\n"));
	}
	else
	{
		QListRemoveItem ( 
			&g_GsaGlobalInfo->SARRecvList,
			ListItem );
	}
	
	SpinLockRelease(&g_GsaGlobalInfo->SARRecvListLock);


	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);

	return FSUCCESS;
}


//
// Timer callback invoked on SAR Recvs
//
void
TimerCallbackRecv(
	IN	void				*TimerContext
	)
{
	GSA_SAR_CONTEXT			*pSarContext;
		
	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, TimerCallbackRecv);

	pSarContext					= TimerContext;

	//ASSERT( pSarContext->CurrSeg != pSarContext->TotalSegs );

#ifdef TEST_SAR_RECV_DISCARD
	_DBG_ERROR(("Timer expired\n"));
#endif
	SpinLockAcquire(&pSarContext->Lock);
	// unfortunately we do not have a retry mechanism for loss
	// of first packet, we would need to resend the original request
	// which is long gone at this point, so we let those return failure
	// to caller
	if (pSarContext->CurrSeg > 0)
	{
		ASSERT(pSarContext->RecvHead != NULL);
		if (++pSarContext->RetryCount < 7)
		{
			IBT_DGRM_ELEMENT		*pDgrm;

			// use 1st valid Dgrm we received for this transaction as template
			// for resend.  Only SAR Header and path information is used
			pDgrm = pSarContext->RecvHead;
			pSarContext->ResendSent = TRUE;
			ResendReqSARRecv(pDgrm, pSarContext->TID, pSarContext->CurrSeg+1);
			// restart timer
			TimerStart( &pSarContext->TimerObj, GSA_SAR_TIME_RECV );
			SpinLockRelease(&pSarContext->Lock);
			goto done;
		}
	}
	SpinLockRelease(&pSarContext->Lock);

	// The datagram has timed out, abort the reassembly.
	_DBG_ERROR (( "SAR Retries Exceeded! <Aborting request>\n" ));
	_DBG_PRINT (_DBG_LVL_SAR_INFO, (
		"SAR msg dropped on retries!\n" ));
		
	// Return the dropped datagram segments to the datagram pool.
	DgrmPoolPut( pSarContext->RecvHead );
	AtomicIncrementVoid( &g_GsaGlobalInfo->SARRecvDropped );

	_DBG_PRINT (_DBG_LVL_SAR_INFO, (
		"*** SAR receive drop count (%d) msgs ***\n",
		g_GsaGlobalInfo->SARRecvDropped ));

	DestroyGsaTimer( pSarContext );

done:
    _DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
}


void
DestroyGsaTimer( 
	IN GSA_SAR_CONTEXT *pSarContext )
{
	// Stop timer before destroy to avoid race conditions
	//TimerStop( &Timer->TimerObj );	

	// Destroy the timer object.
	TimerDestroy( &pSarContext->TimerObj );

	// Remove the GSA timer information from the global SAR recv list.
	RemoveSARListEntry( &pSarContext->ListItem );

	// Destroy the timer's lock.
	SpinLockDestroy(&pSarContext->Lock);
	MemoryDeallocate( pSarContext );
}

#endif // IB1_1
