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


#include "igrowpool.h"
#include "imemory.h"
#include "imath.h"


////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//////////////													   ////////////
//////////////		IMPLEMENTATION OF QUICK COMPOSITE POOL		   ////////////
//////////////													   ////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define QPOOL_MEM_TAG	MAKE_MEM_TAG( i, q, p, l )


//
// Initialize the state of a quick composite pool.
//
void 
QCompPoolInitState(
	IN QCOMP_POOL* const pPool )
{
	ASSERT( pPool );

	MemoryClear( pPool, sizeof(QCOMP_POOL) );

	QListInitState(&pPool->m_FreeList);
	ObjMgrInitState(&pPool->m_ObjMgr);

	pPool->m_State = PoolReset;
}


//
// Initialize a quick composite pool.
//
FSTATUS 
QCompPoolInit(
	IN QCOMP_POOL* const pPool,
	IN const uint32 MinCount,
	IN const uint32* const ComponentSizes,
	IN const uint32 NumComponents,
	IN const uint32 GrowSize,
	IN QCPOOL_CTOR_FUNC pfnConstructor,
	IN QCPOOL_INIT_FUNC pfnInitializer OPTIONAL,
	IN QCPOOL_DTOR_FUNC pfnDestructor OPTIONAL,
	IN void* const Context )
{
	FSTATUS	Status;
	uint32	i;
	ASSERT( pPool );
	ASSERT( NumComponents );
	ASSERT( NumComponents == 1 || pfnConstructor );

	QCompPoolInitState( pPool );

	if( NumComponents != 1 && !pfnConstructor )
		return( FINVALID_SETTING );

	// Allocate the array of component sizes and component pointers all 
	// in one allocation.

	pPool->m_pComponents = (void**)MemoryAllocateAndClear( 
		(sizeof(uint32) + sizeof(void*)) * NumComponents,
		FALSE, QPOOL_MEM_TAG );

	if( !pPool->m_pComponents )
		return( FINSUFFICIENT_MEMORY );

	// Calculate the pointer to the array of pointers, used for callbacks.
	pPool->m_ComponentSizes = (uint32*)(pPool->m_pComponents + NumComponents);

	// Copy the user's sizes into our array for future use.
	MemoryCopy( pPool->m_ComponentSizes, ComponentSizes, 
		sizeof(uint32) * NumComponents );

	// Store the number of sub-objects per object.
	pPool->m_NumComponents = NumComponents;

	// Round up and store the size of the components.
	for( i = 0; i < NumComponents; i++ )
	{
		// We roundup each component size so that all components
		// are aligned on a natural boundary.
		pPool->m_ComponentSizes[i] =
			ROUNDUPP2( pPool->m_ComponentSizes[i], sizeof(uintn) );
	}

	pPool->m_GrowSize = GrowSize;

	// Store callback function pointers.
	pPool->m_pfnCtor = pfnConstructor;	// may be NULL
	pPool->m_pfnInit = pfnInitializer;	// may be NULL
	pPool->m_pfnDtor = pfnDestructor;	// may be NULL
	pPool->m_Context = Context;

	if( !ObjMgrInit( &pPool->m_ObjMgr, FALSE ) )
	{
		// some error occurred
		// user should call CompPoolDestroy()
		return( FINSUFFICIENT_MEMORY );
	}

	if( !QListInit( &pPool->m_FreeList ) )
	{
		// some error occurred
		// user should call CompPoolDestroy()
		return( FINSUFFICIENT_MEMORY );
	}

	// We are now initialized.  We change the initialized flag before
	// growing since the grow function asserts that we are initialized.
	pPool->m_State = PoolReady;

	// so far so good.  Do the allocations.
	if( !MinCount )
		return( FSUCCESS );

	Status = QCompPoolGrow( pPool, MinCount );
	// Trap for error.
	if( Status != FSUCCESS )
		pPool->m_State = PoolInitError;

	return( Status );
}


//
// Destroy a quick composite pool.
//
void 
QCompPoolDestroy( 
	IN QCOMP_POOL* const pPool )
{
	ASSERT( pPool );

	if( pPool->m_State != PoolReset )
	{
		// assert if the user hasn't put everything back in the pool
		// before destroying it
		// if they haven't, then most likely they are still using memory
		// that will be freed, and the destructor will not be called!
		ASSERT( QCompPoolCount(pPool) == pPool->m_NumObjects );

		// call the user's destructor for each object in the pool
		if( pPool->m_pfnDtor )
		{
			while( QListCount( &pPool->m_FreeList ) )
			{
				pPool->m_pfnDtor( QListRemoveHead( &pPool->m_FreeList ), 
					pPool->m_Context );
			}
		}
	}

	QListDestroy( &pPool->m_FreeList );
	ObjMgrDestroy( &pPool->m_ObjMgr );
	MemoryDeallocate(pPool->m_pComponents);
}


//
// Grow the number of objects in a quick composite pool.
//
FSTATUS 
QCompPoolGrow(
	IN QCOMP_POOL* const pPool, 
	IN uint32 ObjCount )
{
	FSTATUS		Status = FSUCCESS;
	uint8		*pObject;
	LIST_ITEM	*pListItem;
	uint32		i, ObjSize;

	ASSERT( pPool );
	ASSERT( pPool->m_State == PoolReady );
	ASSERT( ObjCount );

	// Calculate the size of an object.
	ObjSize = 0;
	for( i = 0; i < pPool->m_NumComponents; i++ )
		ObjSize += pPool->m_ComponentSizes[i];

	// Allocate the buffer for the new objects.
	pObject = (uint8*)ObjMgrAllocate( &pPool->m_ObjMgr, ObjSize * ObjCount );

	// Make sure the allocation succeeded.
	if( !pObject )
		return( FINSUFFICIENT_MEMORY );

	// initialize the new elements and add them to the free list
	while( ObjCount-- )
	{
		// Setup the array of components for the current object.
		pPool->m_pComponents[0] = pObject;
		for( i = 1; i < pPool->m_NumComponents; i++ )
		{
			// Calculate the pointer to the next component.
			pPool->m_pComponents[i] = (uint8*)pPool->m_pComponents[i-1] + 
				pPool->m_ComponentSizes[i-1];
		}

		// call the user's constructor
		// this can NOT fail
		if( pPool->m_pfnCtor )
		{
			pListItem = pPool->m_pfnCtor( pPool->m_pComponents, 
				pPool->m_NumComponents, pPool->m_Context );
			ASSERT( pListItem );
		}
		else
		{
			// If no constructor is provided, assume that the LIST_ITEM
			// is stored at the beginning of the first component.
			pListItem = (LIST_ITEM*)pPool->m_pComponents[0];
			// Set the list item to point to itself.
			QListSetObj( pListItem, pListItem );
		}

		// call the user's initializer
		// this can fail!
		if( pPool->m_pfnInit )
		{
			Status = pPool->m_pfnInit( pListItem, pPool->m_Context );
			if( Status != FSUCCESS )
			{
				// user initialization failed
				// we may have only grown the pool by some partial amount
				// Invoke the destructor for the object that failed initialization.
				if( pPool->m_pfnDtor )
					pPool->m_pfnDtor( pListItem, pPool->m_Context );

				// Return the user's status.
				return( Status );
			}
		}

		// Insert the new item in the free list, traping for failure.
		QListInsertHead( &pPool->m_FreeList, pListItem );

		pPool->m_NumObjects++;

		// move the pointer to the next item
		pObject = (uint8*)pObject + ObjSize; 
	}

	return( Status );
}


//
// Get an object from a quick composite pool.
//
LIST_ITEM* 
QCompPoolGet(
	IN QCOMP_POOL* const pPool )
{
	LIST_ITEM	*pListItem;

	ASSERT( pPool );
	ASSERT( pPool->m_State == PoolReady );

	if( !QCompPoolCount( pPool ) )
	{
		// No object is available.
		// Return NULL if the user does not want automatic growth.
		if( !pPool->m_GrowSize )
			return( NULL );

		// we ran out of elements
		// get more
		QCompPoolGrow( pPool, pPool->m_GrowSize );
		// we may not have gotten everything we wanted but we might have 
		// gotten something.
		if( !QCompPoolCount( pPool ) )
			return( NULL );
	}

	pListItem = QListRemoveHead( &pPool->m_FreeList );
	// OK, at this point we have an object
	ASSERT( pListItem );
	return( pListItem );
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//////////////													   ////////////
//////////////			IMPLEMENTATION OF QUICK GROW POOL		   ////////////
//////////////													   ////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//
// Callback to translate quick composite to quick grow pool
// constructor callback.
//
static LIST_ITEM*
QGrowPoolCtorCb(
	IN void** const pCompArray, 
	IN uint32 NumComponents, 
	IN void* const Context )
{
	QGROW_POOL	*pPool = (QGROW_POOL*)Context;

	ASSERT( pPool );
	ASSERT( pPool->m_pfnCtor );
	ASSERT( NumComponents == 1 );

	return( pPool->m_pfnCtor( pCompArray[0], pPool->m_Context ) );
}

//
// Callback to translate quick composite to quick grow pool
// constructor callback.
//
static FSTATUS
QGrowPoolInitCb(
	IN LIST_ITEM* const pListItem, 
	IN void* const Context )
{
	QGROW_POOL	*pPool = (QGROW_POOL*)Context;

	ASSERT( pPool );
	ASSERT( pPool->m_pfnInit );

	return( pPool->m_pfnInit( pListItem, pPool->m_Context ) );
}

//
// Callback to translate quick composite to quick grow pool
// constructor callback.
//
static void
QGrowPoolDtorCb(
	IN LIST_ITEM* const pListItem, 
	IN void* const Context )
{
	QGROW_POOL	*pPool = (QGROW_POOL*)Context;

	ASSERT( pPool );
	ASSERT( pPool->m_pfnDtor );

	pPool->m_pfnDtor( pListItem, pPool->m_Context );
}


//
// Initializes the state of a quick grow pool.
//
void 
QGrowPoolInitState(
	IN QGROW_POOL* const pPool )
{
	MemoryClear( pPool, sizeof(QGROW_POOL) );

	QCompPoolInitState( &pPool->m_QCPool );
}


//
// Initializes a quick grow pool.
//
FSTATUS 
QGrowPoolInit(
	IN QGROW_POOL* const pPool,
	IN const uint32 MinCount,
	IN const uint32 ObjectSize,
	IN const uint32 GrowSize,
	IN QGPOOL_CTOR_FUNC pfnConstructor OPTIONAL,
	IN QGPOOL_INIT_FUNC pfnInitializer OPTIONAL,
	IN QGPOOL_DTOR_FUNC pfnDestructor OPTIONAL,
	IN void* const Context )
{
	FSTATUS Status;
	ASSERT( pPool );

	QGrowPoolInitState( pPool );

	pPool->m_pfnCtor = pfnConstructor;	// may be NULL
	pPool->m_pfnInit = pfnInitializer;	// may be NULL
	pPool->m_pfnDtor = pfnDestructor;	// may be NULL

	Status = QCompPoolInit( &pPool->m_QCPool, MinCount, &ObjectSize, 
		1, GrowSize,
		pfnConstructor ? QGrowPoolCtorCb : NULL,
		pfnInitializer ? QGrowPoolInitCb : NULL,
		pfnDestructor ? QGrowPoolDtorCb : NULL,
		pPool );

	return( Status );
}


//
// Destroy a quick grow pool.
//
void 
QGrowPoolDestroy(
	IN QGROW_POOL* const pPool )
{
	ASSERT( pPool );
	QCompPoolDestroy( &pPool->m_QCPool );
}


////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//////////////													   ////////////
//////////////			IMPLEMENTATION OF COMPOSITE POOL		   ////////////
//////////////													   ////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define CPOOL_MEM_TAG	MAKE_MEM_TAG( i, c, p, l )


static LIST_ITEM*
CompPoolCtorCb(
	IN void** const pCompArray, 
	IN uint32 NumComponents, 
	IN void* const Context )
{
	COMP_POOL	*pPool = (COMP_POOL*)Context;
	LIST_ITEM	*pListItem;
	ASSERT( pPool );
	
	// Set our pointer to the list item, which is stored at the beginning of
	// the first component.
	pListItem = (LIST_ITEM*)pCompArray[0];
	// Calculate the pointer to the user's first component.
	pCompArray[0] = ((uchar*)pCompArray[0]) + sizeof(LIST_ITEM);

	// Set the object pointer in the list item to point to the first of the
	// user's components.
	QListSetObj( pListItem, pCompArray[0] );

	// Invoke the user's constructor callback.
	if( pPool->m_pfnCtor )
		pPool->m_pfnCtor( pCompArray, NumComponents, pPool->m_Context );

	return( pListItem );
}


static FSTATUS
CompPoolInitCb(
	IN LIST_ITEM* const pListItem, 
	IN void* const Context )
{
	COMP_POOL	*pPool = (COMP_POOL*)Context;
	ASSERT( pPool );
	ASSERT( pPool->m_pfnInit );
	ASSERT( QListObj( pListItem ) );

	// Invoke the user's initializer callback.
	return( pPool->m_pfnInit( QListObj( pListItem ), pPool->m_Context ) );
}


static void
CompPoolDtorCb(
	IN LIST_ITEM* const pListItem, 
	IN void* const Context )
{
	COMP_POOL	*pPool = (COMP_POOL*)Context;
	ASSERT( pPool );
	ASSERT( pPool->m_pfnDtor );
	ASSERT( QListObj( pListItem ) );

	// Invoke the user's destructor callback.
	pPool->m_pfnDtor( QListObj( pListItem ), pPool->m_Context );
}


//
// Initializes the state of a composite pool.
//
void 
CompPoolInitState(
	IN COMP_POOL* const pCPool )
{
	ASSERT( pCPool );

	MemoryClear( pCPool, sizeof(COMP_POOL) );

	QCompPoolInitState( &pCPool->m_QCPool );
}


//
// Initialize a composite pool.
//
FSTATUS 
CompPoolInit(
	IN COMP_POOL* const pCPool,
	IN const uint32 MinCount,
	IN const uint32* const ComponentSizes,
	IN const uint32 NumComponents,
	IN const uint32 GrowSize,
	IN CPOOL_CTOR_FUNC pfnConstructor,
	IN CPOOL_INIT_FUNC pfnInitializer OPTIONAL,
	IN CPOOL_DTOR_FUNC pfnDestructor OPTIONAL,
	IN void* const Context )
{
	FSTATUS	Status;
	uint32	*pCompSizes;

	ASSERT( pCPool );
	ASSERT( NumComponents );
	ASSERT( NumComponents == 1 || pfnConstructor );

	CompPoolInitState( pCPool );

	if( NumComponents != 1 && !pfnConstructor )
		return( FINVALID_SETTING );

	// Allocate the array of component sizes, allocation sizes, and 
	// component pointers all in one allocation.
	pCompSizes = (uint32*)MemoryAllocateAndClear( sizeof(uint32) * NumComponents,
		FALSE, CPOOL_MEM_TAG );

	if( !pCompSizes )
		return( FINSUFFICIENT_MEMORY );

	// Copy the user's component sizes into our local component size array.
	MemoryCopy( pCompSizes, ComponentSizes, sizeof(uint32) * NumComponents );

	// Add the size of the list item to the first component.
	pCompSizes[0] += sizeof(LIST_ITEM);

	// Store callback function pointers.
	pCPool->m_pfnCtor = pfnConstructor;	// may be NULL
	pCPool->m_pfnInit = pfnInitializer;	// may be NULL
	pCPool->m_pfnDtor = pfnDestructor;	// may be NULL
	pCPool->m_Context = Context;

	Status = QCompPoolInit( &pCPool->m_QCPool, MinCount, pCompSizes, NumComponents,
		GrowSize, 
		CompPoolCtorCb,
		pfnInitializer ? CompPoolInitCb : NULL,
		pfnDestructor ? CompPoolDtorCb : NULL,
		pCPool );

	return( Status );
}


//
// Destroys a composite pool.
//
void 
CompPoolDestroy(
	IN COMP_POOL* const pCPool )
{
	ASSERT( pCPool );

	QCompPoolDestroy( &pCPool->m_QCPool );
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//////////////													   ////////////
//////////////				IMPLEMENTATION OF GROW POOL			   ////////////
//////////////													   ////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


static LIST_ITEM*
GrowPoolCtorCb(
	IN void** const ppObj,
	IN uint32 Count,
	IN void* const Context )
{
	GROW_POOL	*pPool = (GROW_POOL*)Context;
	LIST_ITEM	*pListItem;
	ASSERT( pPool );
	ASSERT( ppObj );
	ASSERT( Count == 1 );
	
	// Set our pointer to the list item, which is stored at the beginning of
	// the first component.
	pListItem = (LIST_ITEM*)*ppObj;
	// Calculate the pointer to the user's first component.
	*ppObj = ((uchar*)*ppObj) + sizeof(LIST_ITEM);

	// Set the object pointer in the list item to point to the first of the
	// user's components.
	QListSetObj( pListItem, *ppObj );

	// Invoke the user's constructor callback.
	if( pPool->m_pfnCtor )
		pPool->m_pfnCtor( *ppObj, pPool->m_Context );

	return( pListItem );
}


static FSTATUS
GrowPoolInitCb(
	IN LIST_ITEM* const pListItem, 
	IN void* const Context )
{
	GROW_POOL	*pPool = (GROW_POOL*)Context;
	ASSERT( pPool );
	ASSERT( pPool->m_pfnInit );
	ASSERT( QListObj( pListItem ) );

	// Invoke the user's initializer callback.
	return( pPool->m_pfnInit( QListObj( pListItem ), pPool->m_Context ) );
}


static void
GrowPoolDtorCb(
	IN LIST_ITEM* const pListItem, 
	IN void* const Context )
{
	GROW_POOL	*pPool = (GROW_POOL*)Context;
	ASSERT( pPool );
	ASSERT( pPool->m_pfnDtor );
	ASSERT( QListObj( pListItem ) );

	// Invoke the user's destructor callback.
	pPool->m_pfnDtor( QListObj( pListItem ), pPool->m_Context );
}


//
// Initialize the state of a grow pool.
//
void 
GrowPoolInitState(
	IN GROW_POOL* const pGPool )
{
	ASSERT( pGPool );

	MemoryClear( pGPool, sizeof(GROW_POOL) );
	
	QCompPoolInitState( &pGPool->m_QCPool );
}


//
// Initialize a grow pool.
//
FSTATUS 
GrowPoolInit(
	IN GROW_POOL* const pGPool,
	IN const uint32 MinCount,
	IN uint32 ObjectSize,
	IN const uint32 GrowSize,
	IN GPOOL_CTOR_FUNC pfnConstructor OPTIONAL,
	IN GPOOL_INIT_FUNC pfnInitializer OPTIONAL,
	IN GPOOL_DTOR_FUNC pfnDestructor OPTIONAL,
	IN void* const Context )
{
	FSTATUS	Status;

	ASSERT( pGPool );

	GrowPoolInitState( pGPool );

	// Add the size of the list item to the first component.
	ObjectSize += sizeof(LIST_ITEM);

	// Store callback function pointers.
	pGPool->m_pfnCtor = pfnConstructor;	// may be NULL
	pGPool->m_pfnInit = pfnInitializer;	// may be NULL
	pGPool->m_pfnDtor = pfnDestructor;	// may be NULL
	pGPool->m_Context = Context;

	//
	// We need a constructor in all cases for GrowPool, since
	// the user pointer must be manipulated to hide the prefixed LIST_ITEM
	//
	Status = QCompPoolInit( &pGPool->m_QCPool, MinCount, &ObjectSize, 1,
		GrowSize, 
		GrowPoolCtorCb,
		pfnInitializer ? GrowPoolInitCb : NULL,
		pfnDestructor ? GrowPoolDtorCb : NULL,
		pGPool );

	return( Status );
}


//
// Destroy a grow pool.
//
void 
GrowPoolDestroy(
	IN GROW_POOL* const pGPool )
{
	ASSERT( pGPool );
	QCompPoolDestroy( &pGPool->m_QCPool );
}


