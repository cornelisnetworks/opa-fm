/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_PUBLIC_IRESMGR_H_
#define _IBA_PUBLIC_IRESMGR_H_

#include "iba/public/ilist.h"
#include "iba/public/ireqmgr.h"
#include "iba/public/iobjmgr.h"


#ifdef __cplusplus
extern "C"
{
#endif



/*
 * The free pool object provides an asynchronous free pool of available objects.
 * If no objects are available, it queues a request.  A free pool manages 
 * resources but does not create them.
 */


/*
 * Free pool object definition.
 */
typedef struct _FREE_POOL
{
	/* Available elements in free pool. */
	STACK		m_Stack;			/* DO NOT USE!! */

	/* Pending requests for elements. */
	QUEUE		m_RequestQueue;		/* DO NOT USE!! */

	/* Requests are dynamically allocated and added to the stack. */
	STACK		m_FreeRequestStack;	/* DO NOT USE!! */

	boolean		m_Initialized;		/* DO NOT USE!! */

} FREE_POOL;



/*
 * Function declarations.
 */

/* Create a new free pool. */
void		
FreePoolInitState( 
	IN FREE_POOL* const pFreePool );

boolean		
FreePoolInit(
	IN FREE_POOL* const pFreePool, 
	IN const uint32 MinItems );

void		
FreePoolDestroy( 
	IN FREE_POOL* const pFreePool );

/* Get/free resources. */
boolean		
FreePoolPut( 
	IN FREE_POOL* const pFreePool, 
	IN void* const pObject );

boolean		
FreePoolPutArray( 
	IN FREE_POOL* const pFreePool, 
	IN void* const pArray,
	IN const uint32 ItemCount, 
	IN const uint32 ItemSize );

void*		
FreePoolGet( 
	IN FREE_POOL* const pFreePool);

/* Check the free pool to see if a request can be completed, queue
 * a request, or get a request that can be completed.
 */
uint32		
FreePoolCheck( 
	IN const FREE_POOL* const pFreePool, 
	IN const uint32 Count, 
	IN const boolean PartialOk );

boolean		
FreePoolQueueRequest( 
	IN FREE_POOL* const pFreePool, 
	IN const uint32 Count,
	IN const boolean PartialOk, 
	IN REQ_CALLBACK pfnCallback, 
	IN void* const Context1, 
	IN void* const Context2 );

boolean		
FreePoolDequeueRequest( 
	IN FREE_POOL* const pFreePool,
	OUT REQUEST_OBJECT* const pRequest );

/* Return number of objects in the free pool. */
uint32		
FreePoolFreeCount( 
	IN const FREE_POOL* const pFreePool );

/* Initialize every object in the resource manager with the specified
 * function.
 */
void		
FreePoolApplyFunc( 
	IN const FREE_POOL* const pFreePool, 
	IN LIST_APPLY_FUNC pfnFunc,
	IN void* const Context );



/*
 * The resource manager object manages the allocation, deallocation, and 
 * distribution of objects.
 */


/*
 * Resource manager object definition.
 */
typedef struct _RES_MGR
{
	/* Private data structure used by the resource manager object. */
	/* Users should not access these variables directly. */
	FREE_POOL	m_FreePool;		/* DO NOT USE!! */

	/* These variables are exposed to the user to allow the memory to be */
	/* pinned, registered, etc. */
	uint32		m_ItemCount;	/* DO NOT USE!! */
	void		*m_pArray;		/* DO NOT USE!! */
	uint32		m_ArraySize;	/* DO NOT USE!! */

} RES_MGR;


/*
 * Function declarations.
 */

/* Create a new resource manager. */

/* Initializes, Allocates/frees the resources dynamically. */
void		
ResMgrInitState( 
	IN RES_MGR* const pResMgr );

boolean		
ResMgrInit( 
	IN RES_MGR* const pResMgr, 
	IN const uint32 ObjectCount, 
	IN uint32 ObjectSize, 
	IN const uint32 ByteAlignment, 
	IN const uint32 AlignmentOffset );

void		
ResMgrDestroy( 
	IN RES_MGR* const pResMgr );

/* Get/free resources. */
boolean		
ResMgrPut( 
	IN RES_MGR* const pResMgr, 
	IN void* const pObject );

boolean		
ResMgrPutArray( 
	IN RES_MGR* const pResMgr, 
	IN void* const pArray,
	IN const uint32 ItemCount, 
	IN const uint32 ItemSize );

void*		
ResMgrGet( 
	IN RES_MGR* const pResMgr );

/* Check the free pool to see if a request can be completed, queue
 * a request, or get a request that can be completed.
 */
uint32		
ResMgrCheck( 
	IN const RES_MGR* const pResMgr, 
	IN const uint32 Count, 
	IN const boolean PartialOk );

boolean		
ResMgrQueueRequest( 
	IN RES_MGR* const pResMgr, 
	IN const uint32 Count,
	IN const boolean PartialOk, 
	IN REQ_CALLBACK Callback, 
	IN void* const Context1, 
	IN void* const Context2 );

boolean		
ResMgrDequeueRequest( 
	IN RES_MGR* const pResMgr, 
	OUT REQUEST_OBJECT* const pRequest );

/* Return number of objects in the free pool. */
uint32		
ResMgrFreeCount( 
	IN const RES_MGR* const pResMgr );

/* Initialize every object in the resource manager with the specified
 * function.
 */
void		
ResMgrApplyFunc( 
	IN const RES_MGR* const pResMgr, 
	IN LIST_APPLY_FUNC Func,
	IN void* const Context );

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_IRESMGR_H_ */
