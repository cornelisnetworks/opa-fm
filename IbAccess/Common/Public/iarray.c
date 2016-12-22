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


#include "iarray.h"
#include "imemory.h"
#include "imath.h"

// Make a memory tag of 'iarr' for iarray.
#define ARR_MEM_TAG		MAKE_MEM_TAG( i, a, r, r )


///////////////////////////////////////////////////////////////////////////////
// array access methods
// These methods are an attempt to follow the functionality of the C++ STL.
// The names and behavior are obviously not exact matches, but hopefully,
// close enough to make those familiar with the ANSI standard comfortable.
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// ArrayInitState
// 
// Description:
//	This function performs basic array initialization.  This function cannot 
//	fail.
// 
// Inputs:
//	pArray - Pointer to array
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void 
ArrayInitState(
	IN	ARRAY* const	pArray )
{
	DEBUG_ASSERT( pArray );
	MemoryClear( pArray, sizeof( ARRAY ) );
}



///////////////////////////////////////////////////////////////////////////////
// ArrayInit
// 
// Description:
//	This function initializes the array.  New elements in the array are
//	initialized to NULL.
// 
// Inputs:
//	pArray		- Pointer to array
//	InitialSize	- initial number of elements
//	GrowSize	- number of elements to allocate when incrementally growing
//				the array
//	ElementSize	- size of each element in the array
//	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
// 
// Outputs:
//	None.
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY - array is initialized to zero size
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS
ArrayInit(
	IN	ARRAY* const	pArray,
	IN	const uint32	InitialSize,
	IN	const uint32	GrowSize,
	IN	const uint32	ElementSize,
	IN	const uint32	MemFlags)
{
	FSTATUS	Status;

	DEBUG_ASSERT( pArray );
	DEBUG_ASSERT( GrowSize );
	DEBUG_ASSERT( ElementSize );

	ArrayInitState( pArray );

	pArray->m_GrowSize = GrowSize;
	pArray->m_ElementSize = ElementSize;

	// get the storage needed by the user
	Status = ArraySetSize( pArray, InitialSize, MemFlags, NULL );

	return( Status );
}


///////////////////////////////////////////////////////////////////////////////
// ArrayDestroy
// 
// Description:
//	This function deallocates all allocated memory associated with the array.
//	The array is left initialized to zero size.  This function does not fail.
//	If any Array operations did premptable memory allocate (via MemFlags), this
//	may prempt.
// 
// Inputs:
//	pArray - Pointer to array
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void
ArrayDestroy(
	IN	ARRAY* const	pArray )
{
	DEBUG_ASSERT( pArray );

	// Destroy the page array.
	if( pArray->m_pElements )
		MemoryDeallocate( pArray->m_pElements );

	// reset internal state
	AtomicWrite(&pArray->m_Capacity, 0);
	pArray->m_GrowSize = 0;
	pArray->m_Size = 0;
	pArray->m_pElements = NULL;

	return;
}



///////////////////////////////////////////////////////////////////////////////
// ArraySet
// 
// Description:
//	This function sets the element at the specified index.  The array grows as
//	necessary.
// 
// Inputs:
//	pArray		- Pointer to array.
//	Index		- Index of element to get.
//	pElement	- Pointer to the element to set at the specified index.
//	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
//	ppNewElements - *ppNewElements is potential new storage for array resize
//				must have been returned by ArrayPrepareSet(Index)
// 
// Outputs:
//	ppNewElements - *ppNewElements is set to NULL if ppNewElements was used
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY	The array could not be resized to accomodate the
//							specified element.
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
ArraySet(
	IN	ARRAY* const	pArray, 
	IN	const uint32	Index,
	IN	void* const		pElement,
	IN	const uint32	MemFlags,
	IN OUT void**       ppNewElements OPTIONAL)
{
	FSTATUS	Status;

	DEBUG_ASSERT( pArray );
	DEBUG_ASSERT( pElement );

	// Determine if the array has room for this element.
	if( IB_EXPECT_FALSE(Index >= pArray->m_Size ))
	{
		// Resize to accomodate the given index.
		Status = ArraySetSize( pArray, Index + 1, MemFlags, ppNewElements );
		if (Status != FSUCCESS)
			return( Status );
	}

	// At this point, the array is guaranteed to be big enough
	// Copy the data into the array
	MemoryCopy(ArrayGetPtr( pArray, Index ), pElement, pArray->m_ElementSize);

	return( FSUCCESS );
}


///////////////////////////////////////////////////////////////////////////////
// ArraySet32
// 
// Description:
//	This function sets the element at the specified index.  The array grows as
//	necessary.
//	Optimized for ElementSize of 32 bits
// 
// Inputs:
//	pArray		- Pointer to array.
//	Index		- Index of element to get.
//	Element		- the element to set at the specified index.
//	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
//	ppNewElements - *ppNewElements is potential new storage for array resize
//				must have been returned by ArrayPrepareSet(Index)
// 
// Outputs:
//	ppNewElements - *ppNewElements is set to NULL if ppNewElements was used
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY	The array could not be resized to accomodate the
//							specified element.
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
ArraySet32(
	IN	ARRAY* const	pArray, 
	IN	const uint32	Index,
	IN	uint32			Element,
	IN	const uint32	MemFlags,
	IN OUT void**       ppNewElements OPTIONAL)
{
	FSTATUS	Status;

	DEBUG_ASSERT( pArray );

	// Determine if the array has room for this element.
	if( IB_EXPECT_FALSE(Index >= pArray->m_Size ))
	{
		// Resize to accomodate the given index.
		Status = ArraySetSize( pArray, Index + 1, MemFlags, ppNewElements );
		if (Status != FSUCCESS)
			return( Status );
	}

	// At this point, the array is guaranteed to be big enough
	*(ArrayGetPtr32( pArray, Index )) = Element;

	return( FSUCCESS );
}


///////////////////////////////////////////////////////////////////////////////
// ArraySet64
// 
// Description:
//	This function sets the element at the specified index.  The array grows as
//	necessary.
//	Optimized for ElementSize of 64 bits
// 
// Inputs:
//	pArray		- Pointer to array.
//	Index		- Index of element to get.
//	Element		- the element to set at the specified index.
//	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
//	ppNewElements - *ppNewElements is potential new storage for array resize
//				must have been returned by ArrayPrepareSet(Index)
// 
// Outputs:
//	ppNewElements - *ppNewElements is set to NULL if ppNewElements was used
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY	The array could not be resized to accomodate the
//							specified element.
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
ArraySet64(
	IN	ARRAY* const	pArray, 
	IN	const uint32	Index,
	IN	uint64			Element,
	IN	const uint32	MemFlags,
	IN OUT void**       ppNewElements OPTIONAL)
{
	FSTATUS	Status;

	DEBUG_ASSERT( pArray );

	// Determine if the array has room for this element.
	if( IB_EXPECT_FALSE(Index >= pArray->m_Size ))
	{
		// Resize to accomodate the given index.
		Status = ArraySetSize( pArray, Index + 1, MemFlags, ppNewElements );
		if (Status != FSUCCESS)
			return( Status );
	}

	// At this point, the array is guaranteed to be big enough
	*(ArrayGetPtr64( pArray, Index )) = Element;

	return( FSUCCESS );
}



///////////////////////////////////////////////////////////////////////////////
// ArraySetPointer
// 
// Description:
//	This function sets the element at the specified index.  The array grows as
//	necessary.
//	Optimized for ElementSize of sizeof(void*)
// 
// Inputs:
//	pArray		- Pointer to array.
//	Index		- Index of element to get.
//	Element		- the element to set at the specified index.
//	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
//	ppNewElements - *ppNewElements is potential new storage for array resize
//				must have been returned by ArrayPrepareSet(Index)
// 
// Outputs:
//	ppNewElements - *ppNewElements is set to NULL if ppNewElements was used
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY	The array could not be resized to accomodate the
//							specified element.
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
ArraySetPointer(
	IN	ARRAY* const	pArray, 
	IN	const uint32	Index,
	IN	void*			Element,
	IN	const uint32	MemFlags,
	IN OUT void**       ppNewElements OPTIONAL)
{
	FSTATUS	Status;

	DEBUG_ASSERT( pArray );

	// Determine if the array has room for this element.
	if( IB_EXPECT_FALSE(Index >= pArray->m_Size) )
	{
		// Resize to accomodate the given index.
		Status = ArraySetSize( pArray, Index + 1, MemFlags, ppNewElements );
		if (Status != FSUCCESS)
			return( Status );
	}

	// At this point, the array is guaranteed to be big enough
	*(ArrayGetPtrPointer( pArray, Index )) = Element;

	return( FSUCCESS );
}


///////////////////////////////////////////////////////////////////////////////
// ArraySetCapacity
// 
// Description:
//	This function reserves capacity in the array to the specified number of
//	elements.  This function can only fail if the new capacity is larger than
//	the old capacity.  If the new capacity is less than the old, no action
//	is taken.
// 
// Inputs:
//	pArray		- Pointer to array
//	NewCapacity	- Number of elements reserved in the array
//	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
//	ppNewElements - *ppNewElements is potential new storage for array resize
//				must have been returned by ArrayPrepareSize(Size)
// 
// Outputs:
//	ppNewElements - *ppNewElements is set to NULL if ppNewElements was used
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY - array is left unaltered
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
ArraySetCapacity(
	IN	ARRAY* const	pArray, 
	IN	const uint32	NewCapacity,
	IN	const uint32	MemFlags,
	IN OUT void**		ppNewElements OPTIONAL)
{
	uint32	NewBytes = NewCapacity * pArray->m_ElementSize;
	uint32	OldBytes = AtomicRead(&pArray->m_Capacity) * pArray->m_ElementSize;
	uint8	*pNewElements;

	DEBUG_ASSERT( pArray );

	// Do we have to do anything here?
	if( NewCapacity <= AtomicRead(&pArray->m_Capacity) )
	{
		// Nope
		return( FSUCCESS );
	}

	// Allocate our array.
	if (ppNewElements && *ppNewElements)
	{
		pNewElements = (uint8*)*ppNewElements;
		if( pArray->m_pElements )
		{
			// Copy the old array into the new.
			MemoryCopy( pNewElements, pArray->m_pElements, OldBytes);
		}
		// caller will deallocate *ppNewElements, may need to be in a premptable
		// context, so don't deallocate here
		*ppNewElements = pArray->m_pElements;
	} else {
		pNewElements = (uint8*)MemoryAllocate2AndClear(NewBytes, MemFlags, ARR_MEM_TAG);
		if( !pNewElements )
			return( FINSUFFICIENT_MEMORY );
		if( pArray->m_pElements )
		{
			// Copy the old array into the new.
			MemoryCopy( pNewElements, pArray->m_pElements, OldBytes);
			MemoryDeallocate( pArray->m_pElements );
		}
	}
	
	// Set the new array.
	pArray->m_pElements = pNewElements;

	// clear the new elements
	MemoryClear(pArray->m_pElements+OldBytes, NewBytes-OldBytes);

	// Update the array with the new capactity.
	AtomicWrite(&pArray->m_Capacity, NewCapacity);

	return( FSUCCESS );
}

// compute capacity to grow to as part of enlarging array to given size
static __inline
uint32 ArrayComputeCapacity(
	IN	ARRAY* const	pArray, 
	IN	const uint32	Size)
{
	return ROUNDUP(Size, pArray->m_GrowSize);
}


///////////////////////////////////////////////////////////////////////////////
// ArrayPrepareSize
// 
// Description:
//	This function allocates the storage that would be needed to resize the
//	the array to the specified number of elements.
//	This function can only fail if the new size is larger than
//	the old capacity.
//
//	This allows for premptable array allocation for arrays which must be
//	guarded by spin locks.  For example:
//	if (FSUCCESS != ArrayPrepareSize(&myArray, Size,
//										IBA_MEM_FLAG_PREMPTABLE, &pNewElements))
//		failure return
//	SpinLockAcquire
//	status = ArraySetSize(&myArray, Size, IBA_MEM_FLAG_NONE, &pNewElements);
//	SpinLockRelease
//	if (pNewElements)
//		MemoryDeallocate(pNewElements);
// 
// Inputs:
//	pArray	- Pointer to array
//	Size	- new number of elements in the the array
//	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
// 
// Outputs:
//	ppNewElements - *ppNewElements memory allocated for use in ArraySet or
//				ArraySetSize or ArraySetCapacity
// 
// Returns:
//	FSUCCESS - successfully allocated (or no allocation needed)
//	FINSUFFICIENT_MEMORY	- unable to allocate
//
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
ArrayPrepareSize(
	IN	ARRAY* const	pArray, 
	IN	const uint32	Size,
	IN	const uint32	MemFlags,
	OUT void**			ppNewElements)
{
	DEBUG_ASSERT( pArray );
	DEBUG_ASSERT( ppNewElements );

	*ppNewElements = NULL;
	// Check to see if the requested size requires growth
	if( Size <= AtomicRead(&pArray->m_Capacity) )
		return( FSUCCESS );

	*ppNewElements = MemoryAllocate2AndClear(
						pArray->m_ElementSize*
										ArrayComputeCapacity(pArray, Size),
						MemFlags, ARR_MEM_TAG);
	if( !*ppNewElements )
		return( FINSUFFICIENT_MEMORY );
	return( FSUCCESS );
}


///////////////////////////////////////////////////////////////////////////////
// ArraySetSize
// 
// Description:
//	This function resizes the array to the specified number of
//	elements.  This function can only fail if the new size is larger than
//	the old capacity.
// 
// Inputs:
//	pArray	- Pointer to array
//	Size	- new number of elements in the the array
//	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
//	ppNewElements - *ppNewElements is potential new storage for array resize
//				must have been returned by ArrayPrepareSize(Size)
// 
// Outputs:
//	ppNewElements - *ppNewElements is set to NULL if ppNewElements was used
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY	- array will have at least as many elements
//							as prior to the call.  Use ArrayGetSize()
//							to determine the number of elements
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
ArraySetSize(
	IN	ARRAY* const	pArray, 
	IN	const uint32	Size,
	IN	const uint32	MemFlags,
	IN OUT void**       ppNewElements OPTIONAL)
{
	FSTATUS	Status;

	DEBUG_ASSERT( pArray );

	// Check to see if the requested size is the same as the existing size.
	if( Size == pArray->m_Size )
		return( FSUCCESS );

	// Determine if the array has room for this element.
	if( Size > AtomicRead(&pArray->m_Capacity) )
	{
		Status = ArraySetCapacity( pArray, ArrayComputeCapacity(pArray, Size),
									MemFlags, ppNewElements );
		if( Status != FSUCCESS )
			return( Status );
	}

	// Are we shrinking the array?
	if( Size < pArray->m_Size )
	{
		uint32	NewBytes = Size * pArray->m_ElementSize;
		uint32	OldBytes = pArray->m_Size * pArray->m_ElementSize;

		// zero the removed elements
		MemoryClear(pArray->m_pElements+NewBytes, OldBytes - NewBytes);
	}

	pArray->m_Size = Size;
	return( FSUCCESS );
}


///////////////////////////////////////////////////////////////////////////////
// ArraySetMinSize
// 
// Description:
//	This function resizes the array to the specified number of
//	elements only if the current number of elements in the array is fewer
//	than the specified size.
// 
// Inputs:
//	pArray	- Pointer to array
//	MinSize	- at least the number of elements needed in the array
//	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
//	ppNewElements - *ppNewElements is potential new storage for array resize
//				must have been returned by ArrayPrepareSize(Size)
// 
// Outputs:
//	ppNewElements - *ppNewElements is set to NULL if ppNewElements was used
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY - array is left unaltered
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS ArraySetMinSize(
	IN	ARRAY* const	pArray, 
	IN	const uint32	MinSize,
	IN	const uint32	MemFlags,
	IN OUT void**       ppNewElements OPTIONAL)
{
	DEBUG_ASSERT( pArray );

	if( MinSize > pArray->m_Size )
	{
		// We have to resize the array
		return( ArraySetSize( pArray, MinSize, MemFlags, ppNewElements ) );
	}

	// We didn't have to do anything
	return( FSUCCESS );
}


///////////////////////////////////////////////////////////////////////////////
// ArrayApplyFunc
// 
// Description:
//	This function calls the user supplied function for each item in the array.
// 
//	The called function has the form:
//		void ApplyFunc( uint32 Index, void *pElement, void *Context );
//	where:
//		Index = index of this element
//		pElement = void* at this location in the array
//		Context = user supplied context
// 
// Inputs:
//	pArray		- pointer to array to iterate through
//	pfnCallback	- callback called for each non-NULL element
//	Context		- context to pass to callback function
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void 
ArrayApplyFunc(
	IN	const ARRAY* const	pArray, 
	IN	ARR_APPLY_FUNC		pfnCallback,
	IN	void* const			Context )
{
	uint32	i;
	void	*pElement;

	DEBUG_ASSERT( pArray );
	DEBUG_ASSERT( pfnCallback );

	for( i = 0; i < pArray->m_Size; i++ )
	{
		pElement = ArrayGetPtr( pArray, i );
		pfnCallback( i, pElement, Context );
	}
}


///////////////////////////////////////////////////////////////////////////////
// ArrayFindFromStart
// 
// Description:
//	This function calls the specified function for each item in the array, 
//	starting from the beginning of the array, until the callback function 
//	returns TRUE or every item was processed.  
// 
//	The called function has the form:
//		boolean FindFunc( uint32 Index, void *pElement, void *Context );
//	where:
//		Index = index of this element
//		pElement = pointer to the element at this location in the array
//		Context = user supplied context
// 
// Inputs:
//	pArray		- pointer to array to iterate through
//	pfnCallback	- callback called for each non-NULL element
//	Context		- context to pass to callback function
// 
// Outputs:
//	None.
// 
// Returns:
//	Index value where execution stopped.  If the callback function never
//	returned TRUE, then the return value = the size of the array.
// 
///////////////////////////////////////////////////////////////////////////////
uint32 
ArrayFindFromStart(
	IN	const ARRAY* const	pArray, 
	IN	ARR_FIND_FUNC		pfnCallback,
	IN	void* const			Context )
{
	uint32	i;
	void	*pElement;

	DEBUG_ASSERT( pArray );
	DEBUG_ASSERT( pfnCallback );

	for( i = 0; i < pArray->m_Size; i++ )
	{
		pElement = ArrayGetPtr( pArray, i );
		// Invoke the callback
		if( pfnCallback( i, pElement, Context ) )
			break;
	}
	return( i );
}


///////////////////////////////////////////////////////////////////////////////
// ArrayFindFromEnd
// 
// Description:
//	This function calls the specified function for each item in the array, 
//	starting from the end of the array, until the callback function returns 
//	TRUE or every item was processed.  
// 
//	The called function has the form:
//		boolean FindFunc( uint32 Index, void *pElement, void *Context );
//	where:
//		Index = index of this element
//		pElement = pointer to the element at this location in the array
//		Context = user supplied context
// 
// Inputs:
//	pArray		- pointer to array to iterate through
//	pfnCallback	- callback called for each non-NULL element
//	Context		- context to pass to callback function
// 
// Outputs:
//	None.
// 
// Returns:
//	Index value where execution stopped.  If the callback function never
//	returned TRUE, then the return value = the size of the array.
// 
///////////////////////////////////////////////////////////////////////////////
uint32 
ArrayFindFromEnd(
	IN	const ARRAY* const	pArray, 
	IN	ARR_FIND_FUNC		pfnCallback,
	IN	void* const			Context )
{
	uint32	i;
	void	*pElement;

	DEBUG_ASSERT( pArray );
	DEBUG_ASSERT( pfnCallback );

	i = pArray->m_Size;

	while( i )
	{
		// Get a pointer to the element in the array.
		pElement = ArrayGetPtr( pArray, --i );
		DEBUG_ASSERT( pElement );

		// Invoke the callback for the current element.
		if( pfnCallback( i, pElement, Context ) )
			return i;
	}

	return( pArray->m_Size );
}

///////////////////////////////////////////////////////////////////////////////
// ArrayFindFromIndex
// 
// Description:
//	This function calls the specified function for each item in the array, 
//	starting from the given index of the array, until the callback function 
//	returns TRUE or every item was processed.  
// 
//	The called function has the form:
//		boolean FindFunc( uint32 Index, void *pElement, void *Context );
//	where:
//		Index = index of this element
//		pElement = pointer to the element at this location in the array
//		Context = user supplied context
// 
// Inputs:
//	pArray		- pointer to array to iterate through
//	pfnCallback	- callback called for each non-NULL element
//	Context		- context to pass to callback function
//	Index		- index to start from
// 
// Outputs:
//	None.
// 
// Returns:
//	Index value where execution stopped.  If the callback function never
//	returned TRUE, then the return value = the size of the array.
// 
///////////////////////////////////////////////////////////////////////////////
uint32 
ArrayFindFromIndex(
	IN	const ARRAY* const	pArray, 
	IN	ARR_FIND_FUNC		pfnCallback,
	IN	void* const			Context,
	IN	const uint32	Index
 )
{
	uint32	i;
	void	*pElement;

	DEBUG_ASSERT( pArray );
	DEBUG_ASSERT( pfnCallback );

	// if Index is beyond Size, we will exit this loop immediately
	for( i = Index; i < pArray->m_Size; i++ )
	{
		pElement = ArrayGetPtr( pArray, i );
		// Invoke the callback
		if( pfnCallback( i, pElement, Context ) )
			return( i );
	}

	for( i = 0; i < Index; i++ )
	{
		// Get a pointer to the element in the array.
		pElement = ArrayGetPtr( pArray, i );
		DEBUG_ASSERT( pElement );

		// Invoke the callback for the current element.
		if( pfnCallback( i, pElement, Context ) )
			return i;
	}

	return( pArray->m_Size );
}
