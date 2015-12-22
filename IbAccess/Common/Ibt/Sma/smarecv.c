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

#include "smamain.h"


//
// Forward declarations
//





//
// SmaPostRecv
//
//	The caller can notify the SMA to recieve these many requests on this port. 
//	If the caller is expecting a known number of replies/receives, it should 
//	inform the SMA in advance to queue additional receives so as not to drop 
//	them. This call does not gaurantee that the SMA will not drop all incoming 
//	requests. However, it can help the SMA decide the traffic on this port.
//
//
// INPUTS:
//
//	SmObject		-Object returned in the Open call.
//
//	SmaCaContext	-Context to Channel Adapter.
//
//	PortNumber		-Port on Channel Adapter.
//
//	NumSmps			-Number of Smps that the caller is expecting on port 
//					 specified.
//
//
// OUTPUTS:
//
//
// RETURNS:
//
//
//
FSTATUS
iba_smi_post_recv(
	IN	SMA_OBJECT		*SmObject,
	IN	void			*SmaCaContext,		// Incoming CA
	IN	uint8			PortNumber,			// Incoming Port 
	IN	uint32			NumSmps				// Expected Smp's to be received.
	)
{
	FSTATUS				status = FERROR;
	SMA_CA_OBJ_PRIVATE	*pCaObj;
	POOL_DATA_BLOCK		*pSmpBlock = NULL;
	POOL_DATA_BLOCK		*pSmpBlockNext;
	uint32				smps;
	IB_WORK_REQ			*workRequest;
	SMA_PORT_TABLE_PRIV	*pCaPortTbl=NULL;
	CA_MEM_LIST			caMemList;
	IB_WORK_REQ			wrq;
	IB_LOCAL_DATASEGMENT dsList[2];
	
	size_t				i;
	
	
	_DBG_ENTER_LVL(_DBG_LVL_RECV, iba_smi_post_recv);


	//
	// param check
	//

	//
	// lock
	//
	
	pCaObj = (SMA_CA_OBJ_PRIVATE *)SmaCaContext;

	if ( NULL != pCaObj )
	{
		// get port info
		pCaPortTbl = (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;

		// Only process Qp if not in Destory state
		if ( QP_DESTROY != pCaPortTbl[PortNumber-1].PortBlock->Qp0Control )
		{
			smps = AtomicRead(&pCaPortTbl[PortNumber-1].PortBlock->Qp0RecvQTop);
			if (smps > NumSmps)
				smps = NumSmps;
			// Do not post if the q is full
			if (smps)
			{
				status = iba_smi_pool_get( SmObject, &smps,
											(SMP_BLOCK **)&pSmpBlock);
				if ( FSUCCESS != status )
				{
					_DBG_ERROR(("Not enough memory for SMPs!\n"));
				}
			} else {
				_DBG_PRINT(_DBG_LVL_RECV,(
						"Receive Q is full. No receives posted\n"));
				status = FERROR;
			}
		} else {
			_DBG_ERROR(("Qp in Destroy state!\n"));
			status = FINVALID_STATE;
		}
	} else {
		status = FERROR;	// No Ca
	}


	if ( (NULL != pCaPortTbl) && 
		 (FSUCCESS == status) &&
		 (NULL != pSmpBlock) )
	{
		// Fill in common work request info
		MemoryClear(&wrq, sizeof(wrq));
		workRequest = &wrq;
		workRequest->DSList = &dsList[0];
		workRequest->Operation = WROpRecv;
	

		//
		//BUGBUG: Do this in one go!
		//
		for ( i=0; i<smps; i++ )
		{

			//
			// De-link SMP
			//
			pSmpBlockNext = (POOL_DATA_BLOCK*)pSmpBlock->Block.Next;

			pSmpBlock->Block.Next = pSmpBlock->Block.Prev = NULL;

			//
			// set default return state
			//
			pSmpBlock->Block.Status = FNOT_DONE;


			//
			// Input values to process upon recv
			//

			pSmpBlock->Block.SmaCaContext = pCaObj;
			pSmpBlock->Block.PortNumber = PortNumber;

			//
			// get LKey and Memory handles
			//

			//
			// Create GRH header
			//
			GetMemInfoFromCaMemTable(
									pCaObj,
									g_Sma->GlobalGrh->CaMemIndex,
									&caMemList
									);
			

			workRequest->DSList[0].Address = \
							(uintn) g_Sma->GlobalGrh->VirtualAddr;
			workRequest->DSList[0].Length = sizeof(IB_GRH);
			workRequest->DSList[0].Lkey = caMemList.LKey;


			//
			// Add other params before posting request
			//

			GetMemInfoFromCaMemTable(
									pCaObj,
									pSmpBlock->CaMemIndex,
									&caMemList
									);
			
			//
			// The MAD block 
			//
			workRequest->DSList[1].Address = \
						(uintn)pSmpBlock->Block.Smp;		
			workRequest->DSList[1].Length = sizeof(STL_SMP);
			
			workRequest->DSList[1].Lkey = caMemList.LKey;

			
			workRequest->DSListDepth = 2;
			workRequest->MessageLen = sizeof(IB_GRH) + sizeof(STL_SMP);

			//
			// set Id to request stucture to unload
			//

			ASSERT_VALID_WORKREQID((uintn)pSmpBlock);
			workRequest->WorkReqId		= BUILD_RQ_WORKREQID((uintn)pSmpBlock);
			pCaObj->WorkReqId++;

			//
			// Post on Recv
			//

			AtomicDecrementVoid(&pCaPortTbl[PortNumber-1].PortBlock->Qp0RecvQTop);
					
			_DBG_PRINT(_DBG_LVL_RECV,(
				"PostRecv(0x%p) Next(0x%p) Prev(0x%p)\n",
				_DBG_PTR(pSmpBlock),
				_DBG_PTR(pSmpBlock->Block.Next),
				_DBG_PTR(pSmpBlock->Block.Prev) ));


			SpinLockAcquire( &pCaPortTbl[PortNumber-1].PortBlock->Qp0Lock );
	
			status = iba_post_recv(
						pCaPortTbl[PortNumber-1].PortBlock->QpHandle,
						workRequest
						);
			SpinLockRelease( &pCaPortTbl[PortNumber-1].PortBlock->Qp0Lock );

			//
			// do all SMPs
			//

			pSmpBlock = pSmpBlockNext;
		}
	}
	

	//
	// unlock
	//

	
	_DBG_LEAVE_LVL(_DBG_LVL_RECV);
    
    return status;
}

// SmaRecv
//
//	Returns any SMPs received. If the call is successful, the NumSmps will
//	indicate the number of SMPS received and the SmpBlockList will hold 
//	a list of SmpBlocks received.
//
//
// INPUTS:
//
//	SmObject		-Object returned in the Open call.
//
//	NumSmps			-Number of SMPs received.
//
//	SmpBlockList	-A Linked list of receives with specific information.
//					 The list contains individual SMP_BLOCKs and the
//					 elements in list is indicated by NumSmps.
//
//
// OUTPUTS:
//
//
// RETURNS:
// 	FNOT_DONE - not implemented due to present SMI queing model
// 		hence never finds any data available to return
//
//
//
FSTATUS
iba_smi_recv(
    IN	SMA_OBJECT	*SmObject,
	IN	uint32		*NumSmps,
	OUT	SMP_BLOCK	**SmpBlockList
	)
{
	return FNOT_DONE;
}
