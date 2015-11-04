/* BEGIN_ICS_COPYRIGHT6 ****************************************

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

** END_ICS_COPYRIGHT6   ****************************************/


#include "ireqmgr.h"
#include "imemory.h"

// minimum number of objects to allocate
#define REQ_MGR_START_SIZE 10
// minimum number of objects to grow
#define REQ_MGR_GROW_SIZE 10


///////////////////////////////////////////////////////////////////////////////
// ReqMgrInitState
// 
// Description:
//	This function performs basic request manager initialization.  This 
//	function cannot fail.
// 
// Inputs:
//	pReqMgr - Pointer to a request manager structure.
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void
ReqMgrInitState(
	IN REQ_MGR* const pReqMgr )
{
	ASSERT( pReqMgr );

	// Clear the structure.
	MemoryClear( pReqMgr, sizeof(REQ_MGR) );

	// Initialize the state of the request queue.
	QueueInitState( &pReqMgr->m_RequestQueue );

	// Initialize the state of the free request stack.
	GrowPoolInitState( &pReqMgr->m_RequestPool );
}


///////////////////////////////////////////////////////////////////////////////
// ReqMgrInit
// 
// Description:
//	Initializes the request manager for use.
// 
// Inputs:
//	pReqMgr			- Pointer to a request manager.
//	pfnGetCount		- Pointer to a callback function invoked to 
//					get the number of available objects.
//	GetContext		- Context to pass into the pfnGetCount function.
//
// Outputs:
//	None.
// 
// Returns:
//	FSUCCESS				- the request manager was successfully initialized.
//	FINSUFFICIENT_MEMORY	- there was not enough memory to initialize the
//							request manager.
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS
ReqMgrInit(
	IN REQ_MGR* const pReqMgr,
	IN REQMGR_GET_COUNT_FUNC pfnGetCount,
	IN void* const GetContext )
{
	FSTATUS		Status;

	ASSERT( pReqMgr );
	ASSERT( pfnGetCount );

	// Initialize the state of the request manager, in case it hasn't yet 
	// been done.
	ReqMgrInitState( pReqMgr );

	if( !QueueInit( &pReqMgr->m_RequestQueue, 1 ) )
		return( FINSUFFICIENT_MEMORY );

	Status = GrowPoolInit( &pReqMgr->m_RequestPool, REQ_MGR_START_SIZE, 
		sizeof(REQUEST_OBJECT), REQ_MGR_GROW_SIZE, NULL, NULL, NULL, NULL );

	if( Status != FSUCCESS )
		return( Status );

	// Store callback information for the count function.
	pReqMgr->m_pfnGetCount = pfnGetCount;
	pReqMgr->m_GetContext = GetContext;

	pReqMgr->m_Initialized = TRUE;

	return( FSUCCESS );
}


///////////////////////////////////////////////////////////////////////////////
// ReqMgrDestroy
// 
// Description:
//	Destroys a request manager.
// 
// Inputs:
//	pReqMgr				- Pointer to a request manager.
//
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void
ReqMgrDestroy(
	IN REQ_MGR* const pReqMgr )
{
	REQUEST_OBJECT	*pReq;

	ASSERT( pReqMgr );

	// Return all requests to the grow pool.
	if( pReqMgr->m_Initialized )
	{
		while( NULL != (pReq = (REQUEST_OBJECT*)QueueRemove( &pReqMgr->m_RequestQueue )) )
			GrowPoolPut( &pReqMgr->m_RequestPool, pReq );
	}

	QueueDestroy( &pReqMgr->m_RequestQueue );
	GrowPoolDestroy( &pReqMgr->m_RequestPool );

	pReqMgr->m_Initialized = FALSE;
}


///////////////////////////////////////////////////////////////////////////////
// ReqMgrGet
// 
// Description:
//	Retrieves a number of objects, either synchronously or asynchronously.
// 
// Inputs:
//	pReqMgr		- Pointer to a request manager.
//	pCount		- Contains the number of objects to retrieve.
//	ReqType		- Type of get operation.  Can be ReqGetSync, ReqGetAsync, or
//				ReqGetAsyncPartialOk.
//	pfnCallback	- Pointer to a callback function to store for the request.
//	Context1	- First of two contexts passed to the request callback. 
//	Context2	- Second of two contexts passed to the request callback.
//
// Outputs:
//	pCount		- Contains the number of objects available.
// 
// Returns:
//	FSUCCESS	- The request callback has been invoked with elements to 
//				satisfy the request.  If the request allowed partial 
//				completions, all elements are guaranteed to have been returned.
//	FPENDING	- The request could not complete in it's entirety.  If the 
//				request allowed partial completion, some elements may have been
//				already returned.
//	FINSUFFICIENT_RESOURCES	- There were not enough objects for the request to 
//							succeed.
//	FINSUFFICIENT_MEMORY	- There was not enough memory to process the 
//							request (including queuing the request).
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS
ReqMgrGet(
	IN REQ_MGR* const pReqMgr,
	IN OUT uint32* const pCount,
	IN const REQ_MGR_TYPE ReqType,
	IN REQ_CALLBACK pfnCallback,
	IN void* const Context1,
	IN void* const Context2 )
{
	uint32				AvailableCount;
	uint32				Count;
	REQUEST_OBJECT		*pRequest;

	ASSERT( pReqMgr );
	ASSERT( pReqMgr->m_Initialized );
	ASSERT( pCount );
	ASSERT( *pCount );

	// Get the number of available objects in the grow pool.
	AvailableCount = pReqMgr->m_pfnGetCount( pReqMgr->m_GetContext );

	// Check to see if there is nothing on the queue, and there are 
	// enough items to satisfy the whole request.
	if( !QueueCount( &pReqMgr->m_RequestQueue ) && *pCount <= AvailableCount )
		return( FSUCCESS );

	if( ReqType == ReqGetSync )
		return( FINSUFFICIENT_RESOURCES );

	// We need a request object to place on the request queue.
	pRequest = (REQUEST_OBJECT*)GrowPoolGet( &pReqMgr->m_RequestPool );

	if( !pRequest )
		return( FINSUFFICIENT_MEMORY );

	// We can return the available number of objects but we still need
	// to queue a request for the remainder.
	if( ReqType == ReqGetAsyncPartialOk && 
		!QueueCount( &pReqMgr->m_RequestQueue ) )
	{
		Count = *pCount - AvailableCount;
		*pCount = AvailableCount;
		pRequest->PartialOk = TRUE;
	}
	else
	{
		// We cannot return any objects.  We queue a request for all of them.
		Count = *pCount;
		*pCount = 0;
		pRequest->PartialOk = FALSE;
	}

	// Set the request fields and enqueue it.
	pRequest->pfnCallback = pfnCallback;
	pRequest->Context1 = Context1;
	pRequest->Context2 = Context2;
	pRequest->Count = Count;

	if( !QueueInsert( &pReqMgr->m_RequestQueue, pRequest ) )
	{
		// We could not queue the request.  Return the request to the pool.
		GrowPoolPut( &pReqMgr->m_RequestPool, pRequest );
		return( FINSUFFICIENT_MEMORY );
	}

	return( FPENDING );
}


///////////////////////////////////////////////////////////////////////////////
// ReqMgrResume
// 
// Description:
//	Continues (and completes if possible) a pending request for objects, 
//	if any.
// 
// Inputs:
//	pReqMgr		- Pointer to a request manager.
//
// Outputs:
//	pCount		- Number of objects available for a resuming request.
//	ppfnCallback - Contains the callback function pointer as provided
//				in the ReqMgrGet function.
//	pContext1	- Contains the Context1 value for the resuming request,
//				as provided in the ReqMgrGet function.
//	pContext2	- Contains the Context2 value for the resuming request,
//				as provided in the ReqMgrGet function.
// 
// Returns:
//	FSUCCESS				- A request was completed.
//	FNOT_DONE				- There were no pending requests.
//	FINSUFFICIENT_RESOURCES	- There were not enough resources to complete a
//							pending request.
//	FPENDING				- A request was continued, but not completed.
//
// Remarks:
//	At most one requset is resumed.
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
ReqMgrResume(
	IN REQ_MGR* const pReqMgr,
	OUT uint32* const pCount,
	OUT REQ_CALLBACK* const ppfnCallback,
	OUT void** const pContext1,
	OUT void** const pContext2 )
{
	uint32				AvailableCount;
	REQUEST_OBJECT		*pQueuedRequest;

	ASSERT( pReqMgr );
	ASSERT( pReqMgr->m_Initialized );

	// If no requests are pending, there's nothing to return.
	if( !QueueCount( &pReqMgr->m_RequestQueue ) )
		return FNOT_DONE;

	// Get the item at the head of the request queue, but do not remove it yet.
	pQueuedRequest = (REQUEST_OBJECT*)
		QueueGetHead( &pReqMgr->m_RequestQueue );
	// If no requests are pending, there's nothing to return.
	if (pQueuedRequest == NULL)
		return FNOT_DONE;

	*ppfnCallback = pQueuedRequest->pfnCallback;
	*pContext1 = pQueuedRequest->Context1;
	*pContext2 = pQueuedRequest->Context2;

	AvailableCount = pReqMgr->m_pfnGetCount( pReqMgr->m_GetContext );

	// See if the request can be fulfilled.
	if( pQueuedRequest->Count > AvailableCount )
	{
		if( !pQueuedRequest->PartialOk )
			return( FINSUFFICIENT_RESOURCES );

		pQueuedRequest->Count -= AvailableCount;

		*pCount = AvailableCount;
		return( FPENDING );
	}

	*pCount = pQueuedRequest->Count;

	// The entire request can be met.  Remove it from the request queue
	// and return.
	QueueRemove( &pReqMgr->m_RequestQueue );

	// Return the internal request object to the free stack.
	GrowPoolPut( &pReqMgr->m_RequestPool, pQueuedRequest );
	return( FSUCCESS );
}
