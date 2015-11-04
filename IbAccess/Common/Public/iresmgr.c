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

#include "iresmgr.h"
#include "imemory.h"


#define RES_MGR_TAG		MAKE_MEM_TAG( i, r, e, s )


// Private function declaration used only internally to the free pool.
uint32 
FreePoolGetAvailableCount(
	IN const FREE_POOL* const pFreePool, 
	IN const uint32 Count, 
	IN const boolean PartialOk );


void 
FreePoolInitState(
	IN FREE_POOL* const pFreePool )
{
	StackInitState( &pFreePool->m_Stack );
	StackInitState( &pFreePool->m_FreeRequestStack );
	QueueInitState( &pFreePool->m_RequestQueue );
	pFreePool->m_Initialized = FALSE;
}


boolean 
FreePoolInit(
	IN FREE_POOL* const pFreePool, 
	IN const uint32 MinItems )
{
	uint32		RequestCount;

	ASSERT( pFreePool );

	FreePoolInitState( pFreePool );
	if( !StackInit( &pFreePool->m_Stack, MinItems ) )
		return FALSE;

	// Assume that we may have requests for 25% more resources than the 
	// minimum number of objects that we support.
	if( !(RequestCount = (MinItems >> 2)) )
		RequestCount = 1;

	if( !QueueInit( &pFreePool->m_RequestQueue, RequestCount ) )
	{
		FreePoolDestroy( pFreePool );
		return FALSE;
	}

	if( !StackInit( &pFreePool->m_FreeRequestStack, RequestCount ) )
	{
		FreePoolDestroy( pFreePool );
		return FALSE;
	}

	pFreePool->m_Initialized = TRUE;
	return TRUE;
}


void 
FreePoolDestroy(
	IN FREE_POOL* const pFreePool )
{
	REQUEST_OBJECT	*pRequest;

	ASSERT( pFreePool );

	if( pFreePool->m_Initialized )
	{
		// Deallocate any request objects on the request queue.
		while( NULL != (pRequest = (REQUEST_OBJECT*) QueueRemove( &pFreePool->m_RequestQueue )) )
		{
			MemoryDeallocate( pRequest );
		}

		// Deallocate any request objects on the free request stack.
		while( NULL != (pRequest = (REQUEST_OBJECT*) StackPop( &pFreePool->m_FreeRequestStack )) )
		{
			MemoryDeallocate( pRequest );
		}
	}

	StackDestroy( &pFreePool->m_FreeRequestStack );
	QueueDestroy( &pFreePool->m_RequestQueue );
	StackDestroy( &pFreePool->m_Stack );
	pFreePool->m_Initialized = FALSE;
}


//
// Return the number of items that can be given by the free pool to 
// complete the specified request.  Users of the free pool must cooperate 
// to ensure concurrency.
//
uint32 
FreePoolCheck(
	IN const FREE_POOL* const pFreePool, 
	IN const uint32 Count, 
	IN const boolean PartialOk )
{
	ASSERT( pFreePool && Count && pFreePool->m_Initialized );

	// If anything is on the request queue, we cannot fulfill this request
	// ahead of it.
	if( QueueCount( &pFreePool->m_RequestQueue ) )
		return 0;

	return FreePoolGetAvailableCount( pFreePool, Count, PartialOk );
}


//
// Return the number of elements that may be removed from the free pool.
// This assumes that other checking has been done to verify that elements
// may be successfully removed (e.g. no queued requests)
//
uint32 
FreePoolGetAvailableCount(
	IN const FREE_POOL* const pFreePool, 
	IN const uint32 Count, 
	IN const boolean PartialOk )
{
	// See if we have enough items to complete the entire request.
	if ( StackCount( &pFreePool->m_Stack ) >= Count )
		return Count;

	// We can't complete the entire request, but we may be able to complete
	// part of the request if the user will accept a partial completion.
	// Return the number of items on the free pool.
	if ( PartialOk )
		return StackCount( &pFreePool->m_Stack );
	else
		return 0;
}


//
// Adds an object to the free pool.
//
boolean 
FreePoolPut(
	IN FREE_POOL* const pFreePool, 
	IN void* const pObject )
{
	ASSERT( pFreePool && pFreePool->m_Initialized );
	return StackPush( &pFreePool->m_Stack, pObject );
}


//
// Adds an array of objects to the free pool.
//
boolean 
FreePoolPutArray( 
	IN FREE_POOL* const pFreePool,
	IN void* const pArray,
	IN const uint32 ItemCount, 
	IN const uint32 ItemSize )
{
	ASSERT( pFreePool && pFreePool->m_Initialized );
	return StackPushArray( &pFreePool->m_Stack, pArray, 
		ItemCount, ItemSize );
}


//
// Removes an item from the free list.
//
void* 
FreePoolGet(
	IN FREE_POOL* const pFreePool )
{
	ASSERT( pFreePool && pFreePool->m_Initialized );
	return StackPop( &pFreePool->m_Stack );
}


//
// Queue a request for items from the free pool when they become available.
// The request is queued without checking for available objects.
//
boolean 
FreePoolQueueRequest(
	IN FREE_POOL* const pFreePool, 
	IN const uint32 Count, 
	IN const boolean PartialOk, 
	IN REQ_CALLBACK Callback, 
	IN void* const Context1,
	IN void* const Context2 )
{
	REQUEST_OBJECT		*pRequest;

	ASSERT( pFreePool && pFreePool->m_Initialized );

	// We need a request object to place on the request list.  Try first to
	// get one from the free request list.  If none are available, then we
	// need to allocate a new one.
	if( !(pRequest = (REQUEST_OBJECT*)
		StackPop( &pFreePool->m_FreeRequestStack )) )
	{
		// We need to allocate a new request object.
		pRequest = (REQUEST_OBJECT*)MemoryAllocate( sizeof(REQUEST_OBJECT),
			FALSE, RES_MGR_TAG );
		if( !pRequest )
			return FALSE;
	}

	// Copy the request and enqueue it.
	pRequest->Count = Count;
	pRequest->PartialOk = PartialOk;
	pRequest->pfnCallback = Callback;
	pRequest->Context1 = Context1;
	pRequest->Context2 = Context2;
	if( QueueInsert( &pFreePool->m_RequestQueue, pRequest ) )
	{
		return TRUE;
	}
	else
	{
		// We could not queue the request.  Delete the request object and
		// return.
		if( !StackPush( &pFreePool->m_FreeRequestStack, pRequest ) )
			MemoryDeallocate( pRequest );
		return FALSE;
	}
}


//
// Checks the request list to see if the first request can be completed.
// If it can, information about the request is returned.  If the request
// can only be partially completed, the portion that can be completed is
// returned, and a requests is left queued for the remainder.
//
boolean 
FreePoolDequeueRequest(
	IN FREE_POOL* const pFreePool, 
	OUT REQUEST_OBJECT* const pRequest )
{
	REQUEST_OBJECT		*pQueuedRequest;
	uint32				ItemCount;

	ASSERT( pFreePool && pRequest && pFreePool->m_Initialized );

	// If no requests are pending, there's nothing to return.
	if( !QueueCount( &pFreePool->m_RequestQueue ) )
		return FALSE;

	// Get the item at the head of the request queue, but do not remove it yet.
	pQueuedRequest = (REQUEST_OBJECT*)
		QueueGetHead( &pFreePool->m_RequestQueue );
	// If no requests are pending, there's nothing to return.
	if (pQueuedRequest == NULL)
		return FALSE;

	ItemCount = FreePoolGetAvailableCount( pFreePool, 
			pQueuedRequest->Count, pQueuedRequest->PartialOk );
	// See if the request can be fulfilled.
	if( !ItemCount )
		return FALSE;

	// Return the part of the request than can be fulfilled immediately.
	MemoryCopy( pRequest, pQueuedRequest, sizeof( REQUEST_OBJECT ) );
	if ( pQueuedRequest->Count <= ItemCount )
	{
		// The entire request can be met.  Remove it from the request queue
		// and return.
		QueueRemove( &pFreePool->m_RequestQueue );
		// Return the internal request object to the free stack.
		if( !StackPush( &pFreePool->m_FreeRequestStack, 
			pQueuedRequest ) )
		{
			MemoryDeallocate( pQueuedRequest );
		}
	}
	else
	{
		// We are fulfilling part of the request.  Return the number of items that
		// may be removed from the free pool for the partial completion.
		pRequest->Count = ItemCount;

		// Leave the queued request but update it to reflect the partial completion.
		pQueuedRequest->Count -= ItemCount;
	}
	return TRUE;
}


//
// Return the number of items currently in the free pool.
//
uint32 
FreePoolFreeCount(
	IN const FREE_POOL* const pFreePool )
{
	return StackCount( &pFreePool->m_Stack );
}


//
// Initialize every object in the resource manager with the specified
// function.
//
void 
FreePoolApplyFunc(
	IN const FREE_POOL* const pFreePool, 
	IN LIST_APPLY_FUNC Func,
	IN void* const Context )
{
	ASSERT( pFreePool && pFreePool->m_Initialized );

	// Make sure a function was provided.
	if( !Func )
		return;

	// Invoke the function for each element in the free pool.
	StackApplyFunc( &pFreePool->m_Stack, Func, Context );
}


//
// Asynchronous Resource Manager
//

//
// Allocate the requested resources and add them to the free pool.
// If a byte alignment is specified, the alignment offset into each object
// allocated is aligned on the specified byte boundary.
//
void 
ResMgrInitState(
	IN RES_MGR* const pResMgr )
{
	FreePoolInitState( &pResMgr->m_FreePool );
	pResMgr->m_pArray = NULL;
}

boolean 
ResMgrInit( 
	IN RES_MGR* const pResMgr, 
	IN const uint32 ObjectCount, 
	IN uint32 ObjectSize,
	IN const uint32 ByteAlignment, 
	IN const uint32 AlignmentOffset )
{
	void	*pObject;

	ASSERT( pResMgr && ObjectCount && ObjectSize && 
		AlignmentOffset < ObjectSize );

	ResMgrInitState( pResMgr );

	// Initialize the free pool.
	if( !FreePoolInit( &pResMgr->m_FreePool, ObjectCount ) )
		return FALSE;

	// Allocate the array of objects.  This function will change the object
	// size to reflect any added padding.
	if( !(pResMgr->m_pArray = MemoryAllocateObjectArray( ObjectCount,
		&ObjectSize, ByteAlignment, AlignmentOffset, FALSE, RES_MGR_TAG, 
		&pObject, &pResMgr->m_ArraySize )) )
	{
		ResMgrDestroy( pResMgr );
		return FALSE;
	}

	// Place all elements of the array in the free pool.
	pResMgr->m_ItemCount = ObjectCount;

	// Add the objects to the free pool.
	if( !FreePoolPutArray( &pResMgr->m_FreePool, pObject, 
		ObjectCount, ObjectSize ) )
	{
		// We could not add the objects to the free pool.
		ResMgrDestroy( pResMgr );
		return FALSE;
	}

	return TRUE;
}


//
// Destroy a resource manager.
//
void 
ResMgrDestroy(
	IN RES_MGR* const pResMgr )
{
	ASSERT( pResMgr );

	if( pResMgr->m_pArray )
		MemoryDeallocate( pResMgr->m_pArray );

	FreePoolDestroy( &pResMgr->m_FreePool );
}



//
// Call the free pool for all other operations.
//
boolean 
ResMgrPut( 
	IN RES_MGR* const pResMgr, 
	IN void* const pObject )
{
	ASSERT( pResMgr );

	return FreePoolPut( &pResMgr->m_FreePool, pObject );

}

boolean 
ResMgrPutArray( 
	IN RES_MGR* const pResMgr, 
	IN void* const pArray,
	IN const uint32 ItemCount,
	IN const uint32 ItemSize )
{
	ASSERT( pResMgr );

	return FreePoolPutArray( &pResMgr->m_FreePool, pArray, 
		ItemCount, ItemSize );
}

void* 
ResMgrGet( 
	IN RES_MGR* const pResMgr)
{
	ASSERT( pResMgr );

	return FreePoolGet( &pResMgr->m_FreePool );
}

uint32 
ResMgrCheck(
	IN const RES_MGR* const pResMgr, 
	IN const uint32 Count, 
	IN const boolean PartialOk )
{
	ASSERT( pResMgr );

	return FreePoolCheck( &pResMgr->m_FreePool, Count, PartialOk );
}

boolean 
ResMgrQueueRequest( 
	IN RES_MGR* const pResMgr, 
	IN const uint32 Count,
	IN const boolean PartialOk,
	IN REQ_CALLBACK Callback,
	IN void* const Context1, 
	IN void* const Context2 )
{
	ASSERT( pResMgr );

	return FreePoolQueueRequest( &pResMgr->m_FreePool, Count, 
		PartialOk, Callback, Context1, Context2 );
}

boolean 
ResMgrDequeueRequest(
	IN RES_MGR* const pResMgr,
	OUT REQUEST_OBJECT* const pRequest )
{
	ASSERT( pResMgr );

	return FreePoolDequeueRequest( &pResMgr->m_FreePool, pRequest );
}

uint32 
ResMgrFreeCount( 
	IN const RES_MGR* const pResMgr )
{
	ASSERT( pResMgr );

	return FreePoolFreeCount( &pResMgr->m_FreePool );
}

void 
ResMgrApplyFunc( 
	IN const RES_MGR* const pResMgr, 
	IN LIST_APPLY_FUNC Func,
	IN void *Context )
{
	ASSERT( pResMgr );

	FreePoolApplyFunc( &pResMgr->m_FreePool, Func, Context );
}
