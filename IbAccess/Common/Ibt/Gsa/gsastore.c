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


#define VOID_PTR_INCR(ptr,incr) ptr=((void *)((char *)(ptr)+(incr)))

static FSTATUS DestroyGrowableDgrmPool(IB_HANDLE DgrmPoolHandle);

//
// CreateDgrmPool
//
//
//
// INPUTS:
//
//	ServiceHandle	- Handle returned upon registration
//
//	ElementCount	- Number of elements in pool
//
//	BuffersPerElement- Number of buffers in each pool element
//
//	BufferSizeArray[]- An array of buffers sizes. The length of this array must
//					  be equal to BuffersPerElement.
//
//	ContextSize		- Size of user context in each element
//	
//	
//
// OUTPUTS:
//
//	DgrmPoolHandle	- Opaque handle returned to identify datagram pool created
//
//
//
// RETURNS:
//
//	FSUCCESS
//	FINVALID_PARAMETER
//	FINSUFFICIENT_RESOURCES
//
//

FSTATUS
CreateDgrmPool(
	IN	IB_HANDLE				ServiceHandle,
	IN	uint32					ElementCount,
	IN	uint32                  BuffersPerElement,
	IN	uint32					BufferSizeArray[],
	IN	uint32                  ContextSize,
	OUT IB_HANDLE				*DgrmPoolHandle
	)
{
	FSTATUS						status				= FSUCCESS;
	void						*pUdBlock			= NULL;
	void						*pHeaderBlock		= NULL;
	GLOBAL_MEM_LIST				*pMemList			= NULL;
	IBT_DGRM_ELEMENT			*pPrevDgrmElement	= NULL;
	IBT_BUFFER					*pPrevBuffer		= NULL;
	uint32						memSizeReg			= 0;
	uint32						memSizeHeaders		= 0;

	IBT_MEM_POOL				*dgrmPoolHandle;
	IBT_DGRM_ELEMENT			*pDgrmElement;
	IBT_DGRM_ELEMENT_PRIV		*pIbtDgrmElement;
	IBT_BUFFER					*pBuffer;
	GSA_SERVICE_CLASS_INFO		*serviceClass;
	uint32						allignBytes;
	uint32						i, j;

	_DBG_ENTER_LVL(_DBG_LVL_STORE, CreateDgrmPool);
	
	serviceClass = (GSA_SERVICE_CLASS_INFO *)ServiceHandle;

	// Do per element memory requirement calculations
	// Calculate total buffer sizes
	for (i=0; i<BuffersPerElement; i++)
	{
		memSizeReg += BufferSizeArray[i];
	}


	// IBTA Compliance: MAD packets shall be exactly 256 bytes in length
	// StormLake Compliance: MAD packets shall be up to 2048 bytes in length
	allignBytes = 0;

	if ( NULL != ServiceHandle )		// Not internal call
	{
		// case < MAD_BLOCK_SIZE
		if ( memSizeReg < sizeof(MAD) )
		{
			allignBytes = sizeof(MAD) - memSizeReg;

			_DBG_PRINT(_DBG_LVL_STORE,(
					"Registered memory not a multiple of 256: "
					"Adding (%d) bytes per element of size (%d)\n",
					allignBytes,
					memSizeReg));
		}

		// case > MAD_BLOCK_SIZE: SAR
		if ( memSizeReg > sizeof(MAD) )
		{
			// Each block must contain MAD_BLOCK_SIZE + multiple of 192 bytes

			// TBD - BUGBUG - is this correct given compression
			// have caller supply classHeaderSize as an argument or in
			// class registration
			// may need knowledge of classHeaderSize???
			uint32 classHeaderSize = (serviceClass->MgmtClass == MCLASS_SUBN_ADM)?sizeof(RMPP_HEADER)+sizeof(SA_HDR):sizeof(RMPP_HEADER);

			allignBytes = (sizeof(MAD) - \
							(sizeof(MAD_COMMON) + classHeaderSize)) - \
							((memSizeReg - sizeof(MAD)) %	(sizeof(MAD) - \
							(sizeof(MAD_COMMON) + classHeaderSize)));
		
			_DBG_PRINT(_DBG_LVL_STORE,(
					"Registered memory (SAR) not a multiple of 256+192: "
					"Adding (%d) bytes per element of size (%d)\n",
					allignBytes,
					memSizeReg));
		}

		memSizeReg += allignBytes;
	}

	// Add context and reserved field sizes
	memSizeHeaders += ContextSize;

	memSizeHeaders += (sizeof(IB_LOCAL_DATASEGMENT) * BuffersPerElement);

	// If client is type SAR allocate an addition SAR segment
	if (( NULL != serviceClass ) && ( TRUE == serviceClass->bSARRequired ))
		memSizeHeaders += (sizeof(IB_LOCAL_DATASEGMENT)*2);
	

	memSizeHeaders += ( sizeof(IBT_DGRM_ELEMENT_PRIV) + \
						sizeof(IBT_BUFFER)*BuffersPerElement );
	
	//
	// Now calculate for all elements
	//
	memSizeReg *= ElementCount;
	memSizeHeaders *= ElementCount;

	//
	// Add handle space
	//
	memSizeHeaders += sizeof(IBT_MEM_POOL);

	_DBG_PRINT(_DBG_LVL_STORE,(
		"Mem required: Headers(x%x) Buffers(x%x) for (x%x)Elements\n",
		memSizeHeaders, memSizeReg, ElementCount));

	
	pUdBlock = MemoryAllocate2AndClear(memSizeReg, IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_PREFER_CONTIG, GSA_MEM_TAG);
	if ( NULL != pUdBlock )
	{
		// alloc memory for Headers

		pHeaderBlock = MemoryAllocate2AndClear(memSizeHeaders, IBA_MEM_FLAG_PREMPTABLE, GSA_MEM_TAG);
		if ( NULL != pHeaderBlock )
		{

			// Create global memory block in memory list

			// To avoid a race if AddDevice between CreateGlobalMemList and
			// RegisterMemGlobal, we hold the semaphore to block SmaAddDevice
			// so we don't end up double registering memory as
			// SmaAddDevice will register all MemGlobal blocks with the new CA
			// The AcquireMemListMutex exposure is poor style, ultimately we
			// should move this algorithm into smastore.c as it parallels
			// SmaAllocToBin or better yet create a generic version of the
			// SmaAllocToBin which meets the needs of this and Sma

			AcquireMemListMutex();

			pMemList = CreateGlobalMemList( pUdBlock, memSizeReg,
									pHeaderBlock, TRUE );
			if (NULL == pMemList)
			{
				ReleaseMemListMutex();
			}
		}		// pHeaderBlock
	}			// pUdBlock

	if ( ( NULL != pUdBlock ) && \
		 ( NULL != pHeaderBlock ) && \
		 ( NULL != pMemList ) )
	{
		// set access level for memory region

		pMemList->AccessControl.AsUINT16 = 0;
		pMemList->AccessControl.s.LocalWrite = 1;

		status = RegisterMemGlobal(
								pMemList->VirtualAddr,
								pMemList->Length,
								pMemList->AccessControl,
								&pMemList->CaMemIndex
								);
		ReleaseMemListMutex();

		if ( FSUCCESS == status )
		{
			dgrmPoolHandle = (IBT_MEM_POOL*)pHeaderBlock;

			// Initialize handle
			SpinLockInitState( &dgrmPoolHandle->Lock );
			SpinLockInit( &dgrmPoolHandle->Lock );
	
			dgrmPoolHandle->ServiceHandle = ServiceHandle;
			dgrmPoolHandle->TotalElements = \
				dgrmPoolHandle->Elements = \
				ElementCount;
			VOID_PTR_INCR(pHeaderBlock,sizeof(IBT_MEM_POOL));
			dgrmPoolHandle->DgrmList = (IBT_DGRM_ELEMENT*)pHeaderBlock;
			dgrmPoolHandle->UdBlock = pUdBlock;
			dgrmPoolHandle->HeaderBlock = pHeaderBlock;
			dgrmPoolHandle->MemList = pMemList;
			dgrmPoolHandle->BuffersPerElement = BuffersPerElement;
			dgrmPoolHandle->ReceivePool = FALSE;

			*DgrmPoolHandle = dgrmPoolHandle;	// return value

			// Loop through all elements
			pDgrmElement = (IBT_DGRM_ELEMENT*)pHeaderBlock;
			VOID_PTR_INCR(pHeaderBlock,sizeof(IBT_DGRM_ELEMENT_PRIV));

			// Do each element
			for ( i=0; i< ElementCount; i++ )
			{
				pDgrmElement->Allocated = FALSE;
				pDgrmElement->Element.pBufferList = (IBT_BUFFER*)pHeaderBlock;
				pBuffer = pDgrmElement->Element.pBufferList;

				// Do each buffer
				for ( j=0; j<BuffersPerElement; j++ )
				{
					pBuffer->pData = pUdBlock;
					pBuffer->ByteCount = BufferSizeArray[j];

					VOID_PTR_INCR(pUdBlock,pBuffer->ByteCount);
					
					VOID_PTR_INCR(pHeaderBlock,sizeof(IBT_BUFFER));
					pBuffer->pNextBuffer = (IBT_BUFFER*)pHeaderBlock;

					pPrevBuffer = pBuffer;
					pBuffer = pBuffer->pNextBuffer;
				}	// j loop
				
				// Delink previous buffer
				if (pPrevBuffer)
					pPrevBuffer->pNextBuffer = NULL;

				// Do not update context info if zero
				if ( 0 != ContextSize )
				{
					pDgrmElement->Element.pContext = pHeaderBlock;
					VOID_PTR_INCR(pHeaderBlock,ContextSize);
				}

				// Insert MemHandles
				pIbtDgrmElement = (IBT_DGRM_ELEMENT_PRIV *)pDgrmElement;
				pIbtDgrmElement->MemPoolHandle = dgrmPoolHandle;

				// Add Ds list for quick sends
				pIbtDgrmElement->DsList = (IB_LOCAL_DATASEGMENT *)pHeaderBlock;
				VOID_PTR_INCR(pHeaderBlock,(sizeof(IB_LOCAL_DATASEGMENT) * BuffersPerElement));

				// Skip SAR segment too
				if (( NULL != serviceClass ) && \
					( TRUE == serviceClass->bSARRequired ))
					VOID_PTR_INCR(pHeaderBlock,(sizeof(IB_LOCAL_DATASEGMENT)*2));
	
				// Create next link
				pDgrmElement->Element.pNextElement = (IBT_ELEMENT*)pHeaderBlock;
				VOID_PTR_INCR(pHeaderBlock,sizeof(IBT_DGRM_ELEMENT_PRIV));

				pPrevDgrmElement = pDgrmElement;
				pDgrmElement = \
					(IBT_DGRM_ELEMENT *)pDgrmElement->Element.pNextElement;

				// Add allignment adjustments
				VOID_PTR_INCR(pUdBlock,allignBytes);
			
			}	// i loop

			// Delink next for last element
			if (pPrevDgrmElement)
				pPrevDgrmElement->Element.pNextElement = NULL;

		} else {
			//
			// Remove memory from global list
			//
			DeleteGlobalMemList( pMemList );

			MemoryDeallocate( pHeaderBlock );
			MemoryDeallocate( pUdBlock );

			status = FINSUFFICIENT_RESOURCES;
		}
	} else {
		// errors in allocations. free mem if allocated
		_DBG_ERROR((
			"Could not allocate memory for x%x Elements\n", ElementCount));
		
		// return module specific errors
		status = FINSUFFICIENT_RESOURCES;

		// unwind
		if ( NULL != pHeaderBlock )
			MemoryDeallocate( pHeaderBlock );
		if ( NULL != pUdBlock )
			MemoryDeallocate( pUdBlock );
	}


	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
    return status;
}


//
// DestroyDgrmPool
//
//
//
// INPUTS:
//
//	DgrmPoolHandle	- Handle returned upon successful creation of Datagram pool
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
//	FSUCCESS
//	FINVALID_PARAMETER
//	FINSUFFICIENT_RESOURCES
//
//

FSTATUS
DestroyDgrmPool(
	IN	IB_HANDLE				DgrmPoolHandle
	)
{
	FSTATUS						status = FSUCCESS;
	IBT_MEM_POOL				*dgrmPoolHandle;

	_DBG_ENTER_LVL(_DBG_LVL_STORE, DestroyDgrmPool);

	// Validate
	dgrmPoolHandle = (IBT_MEM_POOL *)DgrmPoolHandle;

	if (dgrmPoolHandle->Growable)
	{
		// growable dgrm pool
		return DestroyGrowableDgrmPool(DgrmPoolHandle);
	}

	if ( dgrmPoolHandle->Elements == dgrmPoolHandle->TotalElements )
	{

		status = DeregisterMemGlobal( dgrmPoolHandle->MemList->CaMemIndex );
		status = DeleteGlobalMemList( dgrmPoolHandle->MemList );
		/* global receive pool destroyed its lock when creating dgrm pool */
		if ( ! dgrmPoolHandle->Parent )
			SpinLockDestroy( &dgrmPoolHandle->Lock );
		MemoryDeallocate(dgrmPoolHandle->UdBlock);
		MemoryDeallocate(dgrmPoolHandle);
	}
	else
	{
		status = FINVALID_PARAMETER;
		_DBG_ERROR((
			"User owns elements!!!\n"
			"\tTotal elements....: %d\n"
			"\tUnfreed elements..: %d\n"
			"\tClass in error....: %x  (%s)\n",  \
			dgrmPoolHandle->TotalElements,
			( dgrmPoolHandle->TotalElements - dgrmPoolHandle->Elements ),
			((GSA_SERVICE_CLASS_INFO*)dgrmPoolHandle->ServiceHandle)->MgmtClass,
			_DBG_PTR(GSI_IS_RESPONDER(
				((GSA_SERVICE_CLASS_INFO*)dgrmPoolHandle->ServiceHandle)->
					RegistrationFlags) ? "ServerClass":"ClientClass")
			));
		
	}

	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
    return status;
}



//
// DgrmPoolGet
//
//
//
// INPUTS:
//
//	DgrmPoolHandle	- Handle returned upon successful creation of Datagram pool
//
//	ElementCount	- Number of elements to fetch from pool
//
//
//	
//
// OUTPUTS:
//
//	ElementCount	- Number of elements fetched from pool
//
//	DgrmList		- Pointer holds the datagram list if successful
//
//
//
// RETURNS:
//
//	FSUCCESS
//	FINVALID_PARAMETER
//	FINSUFFICIENT_RESOURCES
//
//

FSTATUS
DgrmPoolGet(
	IN	IB_HANDLE				DgrmPoolHandle,
	IN OUT	uint32				*ElementCount,
	OUT	IBT_DGRM_ELEMENT		**DgrmList
	)
{
	FSTATUS						status = FSUCCESS;
	IBT_MEM_POOL				*dgrmPoolHandle;
	IBT_DGRM_ELEMENT			*pDgrmList;
	uint32						i;

	_DBG_ENTER_LVL(_DBG_LVL_STORE, DgrmPoolGet);

	dgrmPoolHandle = (IBT_MEM_POOL *)DgrmPoolHandle;

	// Validate input parameters
	if ( !*ElementCount )
	{
		status = FINVALID_PARAMETER;
		_DBG_ERROR(("Client asked for 0 elements!\n"));
	}

	if (FSUCCESS == status)
	{
		SpinLockAcquire( &dgrmPoolHandle->Lock );
		if ( 0 != dgrmPoolHandle->Elements )
		{

			_DBG_PRINT(_DBG_LVL_STORE,("Requested %d Dgrms\n", *ElementCount));

			if ( *ElementCount > dgrmPoolHandle->Elements )
				*ElementCount = dgrmPoolHandle->Elements;
			
			ASSERT(dgrmPoolHandle->DgrmList);

			*DgrmList = dgrmPoolHandle->DgrmList;
			pDgrmList = dgrmPoolHandle->DgrmList;

			if (pDgrmList->Allocated)
				_DBG_ERROR(("GSI ERROR: Allocated Dgrm on Free list\n"));
			if (pDgrmList->OnRecvQ)
				_DBG_ERROR(("GSI ERROR: RecvQ Dgrm on Free list\n"));
			if (pDgrmList->OnSendQ)
				_DBG_ERROR(("GSI ERROR: SendQ Dgrm on Free list\n"));
			if (Gsa_debug_params.debug_level & _DBG_LVL_STORE)
			{
				ASSERT(!pDgrmList->Allocated);
				ASSERT(!pDgrmList->OnRecvQ);
				ASSERT(!pDgrmList->OnSendQ);
			}
			DEBUG_ASSERT(!pDgrmList->Allocated);
			DEBUG_ASSERT(!pDgrmList->OnRecvQ);
			DEBUG_ASSERT(!pDgrmList->OnSendQ);
			pDgrmList->Allocated = TRUE;

			for (i=1; i<*ElementCount; i++ )
			{
				pDgrmList = (IBT_DGRM_ELEMENT *)pDgrmList->Element.pNextElement;
				if (pDgrmList->Allocated)
					_DBG_ERROR(("GSI ERROR: Allocated Dgrm on Free list\n"));
				if (pDgrmList->OnRecvQ)
					_DBG_ERROR(("GSI ERROR: RecvQ Dgrm on Free list\n"));
				if (pDgrmList->OnSendQ)
					_DBG_ERROR(("GSI ERROR: SendQ Dgrm on Free list\n"));
				if (Gsa_debug_params.debug_level & _DBG_LVL_STORE)
				{
					ASSERT(!pDgrmList->Allocated);
					ASSERT(!pDgrmList->OnRecvQ);
					ASSERT(!pDgrmList->OnSendQ);
				}
				DEBUG_ASSERT(!pDgrmList->Allocated);
				DEBUG_ASSERT(!pDgrmList->OnRecvQ);
				DEBUG_ASSERT(!pDgrmList->OnSendQ);
				pDgrmList->Allocated = TRUE;
			}

			dgrmPoolHandle->DgrmList = \
				(IBT_DGRM_ELEMENT *)pDgrmList->Element.pNextElement;
			pDgrmList->Element.pNextElement = NULL;		// Delink
			dgrmPoolHandle->Elements -= *ElementCount;	// update count

			_TRC_STORE(("DgrmPoolGet: Handle %p ElementCount %d Elements %d\n", _TRC_PTR(dgrmPoolHandle), *ElementCount, dgrmPoolHandle->Elements));

			_DBG_PRINT(_DBG_LVL_STORE,
					("[GET] UserDgrmList[x%p]  HandleList[x%p]\n",
					_DBG_PTR(pDgrmList), _DBG_PTR(dgrmPoolHandle->DgrmList)));

			SpinLockRelease( &dgrmPoolHandle->Lock );
		} else {
			SpinLockRelease( &dgrmPoolHandle->Lock );
			status = FINSUFFICIENT_RESOURCES;
			_DBG_PRINT(_DBG_LVL_STORE,("DgrmPoolGet: No more Dgrms in Pool!!!\n"));
		}
	}	// param vaidation

	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
    return status;
}

//
// DgrmPoolCount
//
//
//
// INPUTS:
//
//	DgrmPoolHandle	- Handle returned upon successful creation of Datagram pool
//
// RETURNS:
//
//	Number of elements in the pool
//
uint32
DgrmPoolCount(
	IN	IB_HANDLE				DgrmPoolHandle
	)
{
	IBT_MEM_POOL				*dgrmPoolHandle;
	uint32 rval;

	_DBG_ENTER_LVL(_DBG_LVL_STORE, DgrmPoolCount);
	dgrmPoolHandle = (IBT_MEM_POOL *)DgrmPoolHandle;
	SpinLockAcquire( &dgrmPoolHandle->Lock );
	rval = dgrmPoolHandle != NULL?dgrmPoolHandle->Elements:0;
	SpinLockRelease( &dgrmPoolHandle->Lock );
	_DBG_RETURN_LVL(_DBG_LVL_STORE, rval);
	return rval;
}


//
// DgrmPoolTotal
//
//
//
// INPUTS:
//
//	DgrmPoolHandle	- Handle returned upon successful creation of Datagram pool
//
// RETURNS:
//
//	Total Number of elements allocated in the pool
//
uint32
DgrmPoolTotal(
	IN	IB_HANDLE				DgrmPoolHandle
	)
{
	IBT_MEM_POOL				*dgrmPoolHandle;
	uint32 rval;

	_DBG_ENTER_LVL(_DBG_LVL_STORE, DgrmPoolCount);
	dgrmPoolHandle = (IBT_MEM_POOL *)DgrmPoolHandle;
	SpinLockAcquire( &dgrmPoolHandle->Lock );
	rval = dgrmPoolHandle != NULL?dgrmPoolHandle->TotalElements:0;
	SpinLockRelease( &dgrmPoolHandle->Lock );
	_DBG_RETURN_LVL(_DBG_LVL_STORE, rval);
	return rval;
}


//
// DgrmPoolPut
//
//
//
// INPUTS:
//
//	DgrmList		- Datagram elements to return back to pool
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
//	FSUCCESS
//	FINVALID_PARAMETER
//	FINSUFFICIENT_RESOURCES
//
//

FSTATUS
DgrmPoolPut(
	IN	IBT_DGRM_ELEMENT		*pDgrmList
	)
{
	FSTATUS						status		= FSUCCESS;
	boolean						bRecvPool	= FALSE;

	IBT_MEM_POOL				*dgrmPoolHandle;
	IBT_DGRM_ELEMENT			*p;
	IBT_DGRM_ELEMENT			*pLast = NULL;
	uint32						i;

	_DBG_ENTER_LVL(_DBG_LVL_STORE, DgrmPoolPut);

	ASSERT(pDgrmList);
	dgrmPoolHandle = ((IBT_DGRM_ELEMENT_PRIV *)pDgrmList)->MemPoolHandle;

	if ( TRUE == dgrmPoolHandle->ReceivePool )
		bRecvPool = TRUE;		// Do this to fixup buffer sizes
	// If the buffers are from a growable pool, use the parent handle instead
	if (dgrmPoolHandle->Parent)
		dgrmPoolHandle = dgrmPoolHandle->Parent;

	// Calculate number of returned buffers and cleanup size values in recvs
	p = pDgrmList;
	i=0;	
	while ( NULL != p )
	{
		if (! p->Allocated)
			_DBG_ERROR(("GSI ERROR: Freeing non-allocated Dgrm\n"));
		if (p->OnRecvQ)
			_DBG_ERROR(("GSI ERROR: Freeing RecvQ Dgrm\n"));
		if (p->OnSendQ)
			_DBG_ERROR(("GSI ERROR: Freeing SendQ Dgrm\n"));
		if (Gsa_debug_params.debug_level & _DBG_LVL_STORE)
		{
			ASSERT(p->Allocated);
			ASSERT(!p->OnRecvQ);
			ASSERT(!p->OnSendQ);
		}
		DEBUG_ASSERT(p->Allocated);
		DEBUG_ASSERT(!p->OnRecvQ);
		DEBUG_ASSERT(!p->OnSendQ);
		p->Allocated = FALSE;
		if ( TRUE == bRecvPool )
		{
			p->Element.pBufferList->pNextBuffer->ByteCount =sizeof(MAD);

			// TBD - can we drop the clear here?
			MemoryClear(
				p->Element.pBufferList->pNextBuffer->pData,
				p->Element.pBufferList->pNextBuffer->ByteCount
				);
		}
		pLast = p;
		p = (IBT_DGRM_ELEMENT *)p->Element.pNextElement;
		i++;
	}

	SpinLockAcquire( &dgrmPoolHandle->Lock );

	// add back to head of dgrm pool
	_DBG_PRINT(_DBG_LVL_STORE,("[PUT] (%d) Items to UserDgrmList[x%p]  HandleList[x%p]\n",
				i, _DBG_PTR(pDgrmList), _DBG_PTR(dgrmPoolHandle->DgrmList))); 
	ASSERT(pLast != NULL);
		// &(DgrmList->Element) does not dereference DgrmList
	pLast->Element.pNextElement = &(dgrmPoolHandle->DgrmList->Element);
	dgrmPoolHandle->DgrmList = pDgrmList;
	dgrmPoolHandle->Elements += i;					// total up

	_TRC_STORE(("DgrmPoolPut: Handle %p freed %d Elements %d\n", _TRC_PTR(dgrmPoolHandle), i, dgrmPoolHandle->Elements));

	SpinLockRelease( &dgrmPoolHandle->Lock );

	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
    return status;
}


//
// CreateGrowableDgrmPool
//
//
//
// INPUTS:
//
//	ServiceHandle	- Handle returned upon registration
//
//	ElementCount	- Initial Number of elements in pool (can be 0)
//
//	BuffersPerElement- Number of buffers in each pool element (required)
//
//	BufferSizeArray[]- An array of buffers sizes. The length of this array must
//					  be equal to BuffersPerElement. (required)
//
//	ContextSize		- Size of user context in each element (required)
//	
//	
//
// OUTPUTS:
//
//	DgrmPoolHandle	- Opaque handle returned to identify datagram pool created
//
//
//
// RETURNS:
//
//	FSUCCESS
//	FINVALID_PARAMETER
//	FINSUFFICIENT_RESOURCES
//
//

FSTATUS
CreateGrowableDgrmPool(
	IN	IB_HANDLE				ServiceHandle,
	IN	uint32					ElementCount,
	IN	uint32                  BuffersPerElement,
	IN	uint32					BufferSizeArray[],
	IN	uint32                  ContextSize,
	OUT IB_HANDLE				*DgrmPoolHandle
	)
{
	IBT_MEM_POOL				*dgrmPoolHandle;
	uint32						*pInfo;
	uint32						i;
	FSTATUS						status = FSUCCESS;

	dgrmPoolHandle = (IBT_MEM_POOL*)MemoryAllocate2AndClear(
						sizeof(IBT_MEM_POOL) + sizeof(uint32)
						+ BuffersPerElement*sizeof(uint32),
						IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_PREFER_CONTIG,
						GSA_MEM_TAG);
	if (! dgrmPoolHandle)
		goto failalloc;

	// Initialize handle
	SpinLockInitState( &dgrmPoolHandle->Lock );
	if (! SpinLockInit( &dgrmPoolHandle->Lock ))
		goto faillock;
	dgrmPoolHandle->CallbackItem = SysCallbackGet(dgrmPoolHandle);
	if (! dgrmPoolHandle->CallbackItem)
		goto failsyscall;
	
	dgrmPoolHandle->Next = NULL;
	AtomicWrite(&dgrmPoolHandle->GrowScheduled, 0);
	dgrmPoolHandle->ServiceHandle = ServiceHandle;
	dgrmPoolHandle->TotalElements = dgrmPoolHandle->Elements = 0;
	dgrmPoolHandle->DgrmList = NULL;
	dgrmPoolHandle->UdBlock = NULL;
	dgrmPoolHandle->HeaderBlock = dgrmPoolHandle + 1;// space after IBT_MEM_POOL
	dgrmPoolHandle->MemList = NULL;
	dgrmPoolHandle->BuffersPerElement = BuffersPerElement;
	dgrmPoolHandle->ReceivePool = FALSE;
	dgrmPoolHandle->Growable = TRUE;

	pInfo = (uint32*)dgrmPoolHandle->HeaderBlock;
	*pInfo++ = ContextSize;
	for (i=0; i<BuffersPerElement; ++i)
		*pInfo++ = BufferSizeArray[i];

	if (ElementCount)
	{
		status = DgrmPoolGrow(dgrmPoolHandle, ElementCount);
		if (FSUCCESS != status)
			goto failadd;
	}
	*DgrmPoolHandle = dgrmPoolHandle;	// return value
	return FSUCCESS;

failadd:
	SysCallbackPut(dgrmPoolHandle->CallbackItem);
failsyscall:
	SpinLockDestroy( &dgrmPoolHandle->Lock );
faillock:
	MemoryDeallocate(dgrmPoolHandle);
failalloc:
	return FINSUFFICIENT_RESOURCES;
}

//
// DgrmPoolGrow
// Grow the size of a Growable Dgrm Pool
// This routine may preempt.
//
//
// INPUTS:
//	DgrmPoolHandle	- Opaque handle to datagram pool
//
//	ELementCount	- number of elements to add to pool
//
//	
//
// OUTPUTS:
//
//	None
//
//
// RETURNS:
//
//	FSUCCESS
//	FINVALID_PARAMETER
//	FINSUFFICIENT_RESOURCES
//
FSTATUS
DgrmPoolGrow(
	IN	IB_HANDLE				DgrmPoolHandle,
	IN	uint32					ElementCount
	)
{
	FSTATUS						status				= FSUCCESS;
	IBT_MEM_POOL				*GrowDgrmPool = (IBT_MEM_POOL*)DgrmPoolHandle;
	IBT_MEM_POOL				*dgrmPoolHandle;
	IBT_DGRM_ELEMENT			*pDgrmList;

	_DBG_ENTER_LVL(_DBG_LVL_STORE, DgrmPoolGrow);

	if (! GrowDgrmPool->Growable)
	{
		status = FINVALID_PARAMETER;
		goto done;
	}

	status = CreateDgrmPool(
					GrowDgrmPool->ServiceHandle,
					ElementCount,
					GrowDgrmPool->BuffersPerElement,
					((uint32*)GrowDgrmPool->HeaderBlock)+1,
					*(uint32*)GrowDgrmPool->HeaderBlock, // ContextSize
					(IB_HANDLE *)&dgrmPoolHandle );

	if ( FSUCCESS != status )
	{
		status = FINSUFFICIENT_RESOURCES;
		goto done;
	}

	SpinLockDestroy( &dgrmPoolHandle->Lock );	// we use parent's lock
												// discard this lock 

	// Add RecvQ boolean to DgrmHandle ( this is used in Put )
	dgrmPoolHandle->ReceivePool = GrowDgrmPool->ReceivePool;

	dgrmPoolHandle->Parent = GrowDgrmPool;

	// Now toss it in the global recvQ
	SpinLockAcquire( &GrowDgrmPool->Lock );

	// Attach to Global list
	dgrmPoolHandle->Next = GrowDgrmPool->Next;
	GrowDgrmPool->Next = dgrmPoolHandle;
	
	// Get end of list 
	pDgrmList = dgrmPoolHandle->DgrmList;
	while ( NULL != pDgrmList->Element.pNextElement )
	{
		pDgrmList = (IBT_DGRM_ELEMENT *)pDgrmList->Element.pNextElement;
	}

	pDgrmList->Element.pNextElement = \
		(IBT_ELEMENT *)GrowDgrmPool->DgrmList;		// link

	ASSERT(dgrmPoolHandle->TotalElements == dgrmPoolHandle->Elements);
	GrowDgrmPool->TotalElements += dgrmPoolHandle->TotalElements;
	GrowDgrmPool->Elements += dgrmPoolHandle->TotalElements;

	_TRC_STORE(("Adding %d Total %d To %p\n", dgrmPoolHandle->TotalElements,
					GrowDgrmPool->Elements, _TRC_PTR(GrowDgrmPool)));

	GrowDgrmPool->DgrmList = dgrmPoolHandle->DgrmList;// add to parent Q
	_DBG_INFO(("grew DgrmPool %p by %u now at %u\n",
				_DBG_PTR(GrowDgrmPool), ElementCount,
				GrowDgrmPool->TotalElements));

	SpinLockRelease( &GrowDgrmPool->Lock );

done:
	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
    return status;
}

static FSTATUS
DestroyGrowableDgrmPool(IB_HANDLE DgrmPoolHandle)
{
	IBT_MEM_POOL				*dgrmPoolHandle;
	IBT_MEM_POOL				*GrowDgrmPool = (IBT_MEM_POOL*)DgrmPoolHandle;

	_DBG_ENTER_LVL(_DBG_LVL_STORE, DestroyGrowableDgrmPool);
	
	SpinLockAcquire( &GrowDgrmPool->Lock );
	
	if ( GrowDgrmPool->Elements != GrowDgrmPool->TotalElements ) {
		SpinLockRelease( &GrowDgrmPool->Lock );
		return FINVALID_STATE;
	}

	dgrmPoolHandle = GrowDgrmPool->Next;
	while ( dgrmPoolHandle )
	{
		// force a destroy by reporting all elements are put back in pool

		GrowDgrmPool->Next = dgrmPoolHandle->Next;
		SpinLockRelease( &GrowDgrmPool->Lock );

		dgrmPoolHandle->Elements = dgrmPoolHandle->TotalElements;
		DestroyDgrmPool( dgrmPoolHandle );

		SpinLockAcquire( &GrowDgrmPool->Lock );
		dgrmPoolHandle = GrowDgrmPool->Next;
	}	// while loop

	SpinLockRelease( &GrowDgrmPool->Lock );

	SysCallbackPut(GrowDgrmPool->CallbackItem);
	SpinLockDestroy( &GrowDgrmPool->Lock );
	MemoryDeallocate(GrowDgrmPool);

	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
	return FSUCCESS;
}

// System Callback function to grow Dgrm Pool
static
void
GrowPoolSysCallback(
	void* Key, //Functional Device OBJECT
	void *Context)
{
	IBT_MEM_POOL	*GrowDgrmPool = (IBT_MEM_POOL*)Key;
	uint32 count = (uint32)(uintn)Context;
	FSTATUS status;

	_DBG_ENTER_LVL(_DBG_LVL_STORE, GrowPoolSysCallback);

	_DBG_INFO(("growing DgrmPool %p by %u\n", _DBG_PTR(GrowDgrmPool), count));
	status = DgrmPoolGrow((IB_HANDLE)GrowDgrmPool, count);
	if (status != FSUCCESS) {
		_DBG_WARN(("Unable to grow DgrmPool %p: %s\n", _DBG_PTR(GrowDgrmPool),
								_DBG_PTR(iba_fstatus_msg(status))));
	}
	if (AtomicExchange(&GrowDgrmPool->GrowScheduled, 0) == 0) {
		_DBG_ERROR(("GrowScheduled was 0\n"));
	}

	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
}

//
// DgrmPoolGrowAsNeeded
// Checks Pool size and schedules a system callback to grow it as needed
// This routine does not preempt.
//
//
// INPUTS:
//	DgrmPoolHandle	- Opaque handle to datagram pool
//
//	lowWater		- if Dgrm Pool available < this, we will grow it
//
//	maxElements		- limit on size of Dgrm Pool, will not grow beyond this
//
//	growIncrement	- amount to grow by if we are growing
//
//	
//
// OUTPUTS:
//
//	None
//
//
// RETURNS:
//
//	None
//
void
DgrmPoolGrowAsNeeded(IB_HANDLE DgrmPoolHandle, uint32 lowWater,
				uint32 maxElements, uint32 growIncrement)
{
	IBT_MEM_POOL	*GrowDgrmPool = (IBT_MEM_POOL*)DgrmPoolHandle;

	_DBG_ENTER_LVL(_DBG_LVL_STORE, DgrmPoolGrowAsNeeded);
	if (DgrmPoolCount(DgrmPoolHandle) < lowWater
		&& DgrmPoolTotal(DgrmPoolHandle) < maxElements)
	{
		if (AtomicExchange(&GrowDgrmPool->GrowScheduled, 1) == 0) {
			// now we have lock, double check Total
			uint32 count = DgrmPoolTotal(DgrmPoolHandle);
			if (count < maxElements)
			{
				count = maxElements - count;	// max to grow
				count = MIN(count, growIncrement);
				SysCallbackQueue(GrowDgrmPool->CallbackItem,
									GrowPoolSysCallback,
									(void*)(uintn)count, FALSE);
			} else {
				/* release lock */
				if (AtomicExchange(&GrowDgrmPool->GrowScheduled, 0) == 0) {
					_DBG_ERROR(("RecvQGrowScheduled was 0\n"));
				}
			}
		}
	}
	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
}

// create the global GSA RecvQ Pool
FSTATUS
CreateGlobalRecvQ(void)
{
	FSTATUS						status				= FSUCCESS;
	uint32						bufferSizeArray[2]	= {sizeof(IB_GRH),sizeof(MAD)};

	_DBG_ENTER_LVL(_DBG_LVL_STORE, CreateGlobalRecvQ);

	status = CreateGrowableDgrmPool(
					NULL,
					0,
					2,							// BuffersPerElement
					bufferSizeArray,
					0,							// ContextSize
					&g_GsaGlobalInfo->DgrmPoolRecvQ);
	if (FSUCCESS == status)
		((IBT_MEM_POOL*)g_GsaGlobalInfo->DgrmPoolRecvQ)->ReceivePool = TRUE;

	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
    return status;
}

// grow the global GSA RecvQ Pool
FSTATUS
DgrmPoolAddToGlobalRecvQ(
	IN	uint32					ElementCount
	)
{
	FSTATUS status = FSUCCESS;

	if (DgrmPoolCount(g_GsaGlobalInfo->DgrmPoolRecvQ) <= ElementCount)
		status = DgrmPoolGrow(g_GsaGlobalInfo->DgrmPoolRecvQ, ElementCount);
	return status;
}

//
// GrowRecvQAsNeeded
// Checks RecvQ size and schedules a system callback to grow it as needed
//
//
//
// INPUTS:
//
//	None
//
//	
//
// OUTPUTS:
//
//	None
//
//
// RETURNS:
//
//	None
//
void
GrowRecvQAsNeeded(void)
{
	DgrmPoolGrowAsNeeded(g_GsaGlobalInfo->DgrmPoolRecvQ, GSA_RECVQ_LOW_WATER,
							g_GsaSettings.MaxRecvBuffers,
							g_GsaSettings.PreAllocRecvBuffersPerPort);
}

//
// DestroyGlobalRecvQ
//
//
//
// INPUTS:
//
//	None
//
//	
//
// OUTPUTS:
//
//	None
//
//
// RETURNS:
//
//	None
//
//

void
DestroyGlobalRecvQ(void)
{
	_DBG_ENTER_LVL(_DBG_LVL_STORE, DestroyGlobalRecvQ);

	DestroyDgrmPool(g_GsaGlobalInfo->DgrmPoolRecvQ);

	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
}
