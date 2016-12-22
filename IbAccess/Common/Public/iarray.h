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


#ifndef _IBA_PUBLIC_IARRAY_H_
#define _IBA_PUBLIC_IARRAY_H_

#include "iba/public/datatypes.h"
#include "iba/public/imemory.h"
#include "iba/public/ispinlock.h"


#ifdef __cplusplus
extern "C"
{
#endif

/* Callback function declarations. */
typedef void (*ARR_APPLY_FUNC)( 
	IN const uint32 Index, 
	IN void* const Elem,
	IN void* const Context );

typedef boolean (*ARR_FIND_FUNC)(
	IN const uint32 Index, 
	IN void* const Elem,
	IN void* const Context );

/****************************************************************************
 *
 * Array definition.
 *
 * Array statically stores items in an array, providing constant access 
 * times.  The array can be resized dynamically.  When the array is resized, 
 * objects in the array are relocated in memory.
 *
 * The array is an efficient dynamic storage for simple elements.  If persistent
 * pointers to the elements in the array are required, a VECTOR should be
 * used instead.
 */


/* The array structure is allocated by the user and is opaque to them */
typedef struct _ARRAY
{ 
	/* fields are PRIVATE and should NOT be used directly by callers */
	uint32			m_Size;		/* # of elements logically in the array. */
	uint32			m_GrowSize;	/* # of elements to grab when growing. */
	uint32			m_ElementSize; /* Size of each element. */
	ATOMIC_UINT		m_Capacity;	/* total # of elements allocated. */
								/* This is atomic only so ArrayPrepareSize */
								/* can be called without holding locks */
	uint8			*m_pElements; /* Internal array of elements. */

} ARRAY;



/****************************************************************************
 * array access methods
 * These methods are an attempt to follow the functionality of the C++ STL.
 * The names and behavior are obviously not exact matches, but hopefully,
 * close enough to make those familiar with the ANSI standard comfortable.
 ****************************************************************************/


/****************************************************************************
 * ArrayInitState
 * 
 * Description:
 *	This function performs basic array initialization.  This function cannot 
 *	fail.
 * 
 * Inputs:
 *	pArray - Pointer to array
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void 
ArrayInitState(
	IN	ARRAY* const	pArray );


/****************************************************************************
 * ArrayInit
 * 
 * Description:
 *	This function initializes the array.  New elements in the array are
 *	initialized to 0.
 * 
 * Inputs:
 *	pArray		- Pointer to array
 *	InitialSize	- initial number of elements
 *	GrowSize	- number of elements to allocate when incrementally growing
 *				the array
 *	ElementSize	- size of each element in the array
 *	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY - array is initialized to zero size
 * 
 ****************************************************************************/
FSTATUS
ArrayInit(
	IN	ARRAY* const	pArray,
	IN	const uint32	InitialSize,
	IN	const uint32	GrowSize,
	IN	const uint32	ElementSize,
	IN	const uint32	MemFlags );


/****************************************************************************
 * ArrayDestroy
 * 
 * Description:
 *	This function deallocates all allocated memory associated with the array.
 *	The array is left initialized to zero size.  This function does not fail.
 *	If any Array operations did premptable memory allocate (via MemFlags), this
 *	may prempt.
 * 
 * Inputs:
 *	pArray - Pointer to array
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void
ArrayDestroy(
	IN	ARRAY* const	pArray );


/****************************************************************************
 * ArrayGetCapacity
 *
 * Description:
 *	This function returns the current capacity of the array. 
 *	This function cannot fail.
 *
 * Inputs:
 *	pArray	- Pointer to array
 *
 * Outputs:
 *	None.
 *
 * Returns:
 *	Capacity, in objects, of the array.
 *
 ****************************************************************************/
static __inline uint32
ArrayGetCapacity(
	IN	const ARRAY* const	pArray )
{
	DEBUG_ASSERT( pArray );
	return( AtomicRead(&pArray->m_Capacity) );
}


/****************************************************************************
 * ArrayGetSize
 *
 * Description:
 *	This function returns the size of the array. This function cannot fail.
 *
 * Inputs:
 *	pArray	- Pointer to array
 *
 * Outputs:
 *	None.
 *
 * Returns:
 *	Size, in objects, of the array.
 *
 ****************************************************************************/
static __inline uint32
ArrayGetSize(
	IN	const ARRAY* const	pArray )
{
	DEBUG_ASSERT( pArray );
	return( pArray->m_Size );
}


/****************************************************************************
 * ArrayGetPtr
 * 
 * Description:
 *	This function returns a pointer to the element at the specified index.
 *	Range checking is NOT performed.  Provides constant time access.
 * 
 * Inputs:
 *	pArray	- Pointer to array.
 *	Index	- Index of element to get.
 * 
 * Outputs:
 *	Pointer to the element.
 * 
 * Returns:
 *	None.
 * 
 * Notes:
 *	Pointers retrieved are not valid if the array is resized.
 * 
 ****************************************************************************/
static __inline void* 
ArrayGetPtr(
	IN	const ARRAY* const	pArray, 
	IN	const uint32		Index )
{
	return( pArray->m_pElements + (Index*pArray->m_ElementSize) );
}


/****************************************************************************
 * ArrayGet
 * 
 * Description:
 *	This function returns the element at the specified index.  The returned
 *	element is a copy of the element in the array.  Range checking is NOT 
 *	performed.  Provides constant time access.
 * 
 * Inputs:
 *	pArray		- Pointer to array.
 *	Index		- Index of element to get.
 *	pElement	- Pointer to storage for the element to be returned.
 * 
 * Outputs:
 *	pElement	- Copy of the desired element.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
static __inline void 
ArrayGet(
	IN	const ARRAY* const	pArray,
	IN	const uint32		Index,
	OUT	void * const		pElement )
{
	DEBUG_ASSERT( pArray );
	DEBUG_ASSERT( pElement );

	/* Get a pointer to the element. */
	MemoryCopy(pElement, ArrayGetPtr( pArray, Index ), pArray->m_ElementSize );
}

/****************************************************************************
 * ArrayAt
 * 
 * Description:
 *	This function returns the element at the specified index.  The returned
 *	element is a copy of the element in the array.  Range checking is 
 *	performed.  Provides constant time access.
 * 
 * Inputs:
 *	pArray		- Pointer to array.
 *	Index		- Index of element to get.
 *	pElement	- Pointer to storage for the element to be returned.
 * 
 * Outputs:
 *	pElement	- Copy of the desired element.
 * 
 * Returns:
 *	FSUCCESS
 *	FINVALID_PARAMETER	Index out of range.
 * 
 ****************************************************************************/
static __inline 
FSTATUS 
ArrayAt(
	IN	const ARRAY* const	pArray,
	IN	const uint32		Index,
	OUT	void* const			pElement )
{
	DEBUG_ASSERT( pArray );	/* Other asserts are in ArrayGet */

	/* Range check */
	if( IB_EXPECT_FALSE(Index >= pArray->m_Size ))
		return( FINVALID_PARAMETER );

	ArrayGet( pArray, Index, pElement );
	return( FSUCCESS );
}



/****************************************************************************
 * ArraySet
 * 
 * Description:
 *	This function sets the element at the specified index.  The array grows as
 *	necessary.
 * 
 * Inputs:
 *	pArray		- Pointer to array.
 *	Index		- Index of element to get.
 *	pElement	- Pointer to the element to set at the specified index.
 *	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
 *	ppNewElements - *ppNewElements is potential new storage for array resize
 *				must have been returned by ArrayPrepareSet(Index)
 * 
 * Outputs:
 *	ppNewElements - *ppNewElements is set to NULL if ppNewElements was used
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY	The array could not be resized to accomodate the
 *							specified element.
 * 
 ****************************************************************************/
FSTATUS 
ArraySet(
	IN	ARRAY* const	pArray, 
	IN	const uint32	Index,
	IN	void* const		pElement,
	IN	const uint32	MemFlags,
	IN OUT void**		ppNewElements OPTIONAL);


/****************************************************************************
 * ArrayGetPtr32
 * 
 * Description:
 *	This function returns a pointer to the element at the specified index.
 *	Range checking is NOT performed.  Provides constant time access.
 *	Optimized for ElementSize of 32 bits
 * 
 * Inputs:
 *	pArray	- Pointer to array.
 *	Index	- Index of element to get.
 * 
 * Outputs:
 *	Pointer to the element.
 * 
 * Returns:
 *	None.
 * 
 * Notes:
 *	Pointers retrieved are not valid if the array is resized.
 * 
 ****************************************************************************/
static __inline uint32* 
ArrayGetPtr32(
	IN	const ARRAY* const	pArray, 
	IN	const uint32		Index )
{
	DEBUG_ASSERT(pArray->m_ElementSize == sizeof(uint32));
	return( (uint32*)pArray->m_pElements + Index );
}


/****************************************************************************
 * ArrayGet32
 * 
 * Description:
 *	This function returns the element at the specified index.  The returned
 *	element is a copy of the element in the array.  Range checking is NOT 
 *	performed.  Provides constant time access.
 *	Optimized for ElementSize of 32 bits
 * 
 * Inputs:
 *	pArray		- Pointer to array.
 *	Index		- Index of element to get.
 * 
 * Outputs:
 * 
 * Returns:
 *	copy of desired element
 * 
 ****************************************************************************/
static __inline uint32 
ArrayGet32(
	IN	const ARRAY* const	pArray,
	IN	const uint32		Index )
{
	DEBUG_ASSERT( pArray );

	return *(ArrayGetPtr32( pArray, Index ));
}


/****************************************************************************
 * ArrayAt32
 * 
 * Description:
 *	This function returns the element at the specified index.  The returned
 *	element is a copy of the element in the array.  Range checking is 
 *	performed.  Provides constant time access.
 *	Optimized for ElementSize of 32 bits
 * 
 * Inputs:
 *	pArray		- Pointer to array.
 *	Index		- Index of element to get.
 *	pElement	- Pointer to storage for the element to be returned.
 * 
 * Outputs:
 *	pElement	- Copy of the desired element.
 * 
 * Returns:
 *	FSUCCESS
 *	FINVALID_PARAMETER	Index out of range.
 * 
 ****************************************************************************/
static __inline 
FSTATUS 
ArrayAt32(
	IN	const ARRAY* const		pArray,
	IN	const uint32			Index,
	OUT	uint32* const			pElement )
{
	DEBUG_ASSERT( pArray );	/* Other asserts are in ArrayGet */

	/* Range check */
	if( IB_EXPECT_FALSE(Index >= pArray->m_Size ))
		return( FINVALID_PARAMETER );

	*pElement = ArrayGet32( pArray, Index);
	return( FSUCCESS );
}



/****************************************************************************
 * ArraySet32
 * 
 * Description:
 *	This function sets the element at the specified index.  The array grows as
 *	necessary.
 *	Optimized for ElementSize of 32 bits
 * 
 * Inputs:
 *	pArray		- Pointer to array.
 *	Index		- Index of element to get.
 *	Element		- the element to set at the specified index.
 *	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
 *	ppNewElements - *ppNewElements is potential new storage for array resize
 *				must have been returned by ArrayPrepareSet(Index)
 * 
 * Outputs:
 *	ppNewElements - *ppNewElements is set to NULL if ppNewElements was used
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY	The array could not be resized to accomodate the
 *							specified element.
 * 
 ****************************************************************************/
FSTATUS 
ArraySet32(
	IN	ARRAY* const	pArray, 
	IN	const uint32	Index,
	IN	uint32			Element,
	IN	const uint32	MemFlags,
	IN OUT void**		ppNewElements OPTIONAL);


/****************************************************************************
 * ArrayGetPtr64
 * 
 * Description:
 *	This function returns a pointer to the element at the specified index.
 *	Range checking is NOT performed.  Provides constant time access.
 *	Optimized for ElementSize of 64 bits
 * 
 * Inputs:
 *	pArray	- Pointer to array.
 *	Index	- Index of element to get.
 * 
 * Outputs:
 *	Pointer to the element.
 * 
 * Returns:
 *	None.
 * 
 * Notes:
 *	Pointers retrieved are not valid if the array is resized.
 * 
 ****************************************************************************/
static __inline uint64* 
ArrayGetPtr64(
	IN	const ARRAY* const	pArray, 
	IN	const uint32		Index )
{
	DEBUG_ASSERT(pArray->m_ElementSize == sizeof(uint64));
	return( (uint64*)pArray->m_pElements + Index );
}


/****************************************************************************
 * ArrayGet64
 * 
 * Description:
 *	This function returns the element at the specified index.  The returned
 *	element is a copy of the element in the array.  Range checking is NOT 
 *	performed.  Provides constant time access.
 *	Optimized for ElementSize of 64 bits
 * 
 * Inputs:
 *	pArray		- Pointer to array.
 *	Index		- Index of element to get.
 * 
 * Outputs:
 * 
 * Returns:
 *	copy of desired element
 * 
 ****************************************************************************/
static __inline uint64 
ArrayGet64(
	IN	const ARRAY* const	pArray,
	IN	const uint32		Index )
{
	DEBUG_ASSERT( pArray );

	return *(ArrayGetPtr64( pArray, Index ));
}


/****************************************************************************
 * ArrayAt64
 * 
 * Description:
 *	This function returns the element at the specified index.  The returned
 *	element is a copy of the element in the array.  Range checking is 
 *	performed.  Provides constant time access.
 *	Optimized for ElementSize of 64 bits
 * 
 * Inputs:
 *	pArray		- Pointer to array.
 *	Index		- Index of element to get.
 *	pElement	- Pointer to storage for the element to be returned.
 * 
 * Outputs:
 *	pElement	- Copy of the desired element.
 * 
 * Returns:
 *	FSUCCESS
 *	FINVALID_PARAMETER	Index out of range.
 * 
 ****************************************************************************/
static __inline 
FSTATUS 
ArrayAt64(
	IN	const ARRAY* const		pArray,
	IN	const uint32			Index,
	OUT	uint64* const			pElement )
{
	DEBUG_ASSERT( pArray );	/* Other asserts are in ArrayGet */

	/* Range check */
	if( IB_EXPECT_FALSE(Index >= pArray->m_Size ))
		return( FINVALID_PARAMETER );

	*pElement = ArrayGet64( pArray, Index);
	return( FSUCCESS );
}




/****************************************************************************
 * ArraySet64
 * 
 * Description:
 *	This function sets the element at the specified index.  The array grows as
 *	necessary.
 *	Optimized for ElementSize of 64 bits
 * 
 * Inputs:
 *	pArray		- Pointer to array.
 *	Index		- Index of element to get.
 *	Element		- the element to set at the specified index.
 *	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
 *	ppNewElements - *ppNewElements is potential new storage for array resize
 *				must have been returned by ArrayPrepareSet(Index)
 * 
 * Outputs:
 *	ppNewElements - *ppNewElements is set to NULL if ppNewElements was used
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY	The array could not be resized to accomodate the
 *							specified element.
 * 
 ****************************************************************************/
FSTATUS 
ArraySet64(
	IN	ARRAY* const	pArray, 
	IN	const uint32	Index,
	IN	uint64			Element,
	IN	const uint32	MemFlags,
	IN OUT void**		ppNewElements OPTIONAL);


/****************************************************************************
 * ArrayGetPtrPointer
 * 
 * Description:
 *	This function returns a pointer to the element at the specified index.
 *	Range checking is NOT performed.  Provides constant time access.
 *	Optimized for ElementSize of sizeof(void*)
 * 
 * Inputs:
 *	pArray	- Pointer to array.
 *	Index	- Index of element to get.
 * 
 * Outputs:
 *	Pointer to the element.
 * 
 * Returns:
 *	None.
 * 
 * Notes:
 *	Pointers retrieved are not valid if the array is resized.
 * 
 ****************************************************************************/
static __inline void** 
ArrayGetPtrPointer(
	IN	const ARRAY* const	pArray, 
	IN	const uint32		Index )
{
	DEBUG_ASSERT(pArray->m_ElementSize == sizeof(void*));
	return( (void**)pArray->m_pElements + Index );
}


/****************************************************************************
 * ArrayGetPointer
 * 
 * Description:
 *	This function returns the element at the specified index.  The returned
 *	element is a copy of the element in the array.  Range checking is NOT 
 *	performed.  Provides constant time access.
 *	Optimized for ElementSize of sizeof(void*)
 * 
 * Inputs:
 *	pArray		- Pointer to array.
 *	Index		- Index of element to get.
 * 
 * Outputs:
 * 
 * Returns:
 *	copy of desired element
 * 
 ****************************************************************************/
static __inline void* 
ArrayGetPointer(
	IN	const ARRAY* const	pArray,
	IN	const uint32		Index )
{
	DEBUG_ASSERT( pArray );

	return *(ArrayGetPtrPointer( pArray, Index ));
}


/****************************************************************************
 * ArrayAtPointer
 * 
 * Description:
 *	This function returns the element at the specified index.  The returned
 *	element is a copy of the element in the array.  Range checking is 
 *	performed.  Provides constant time access.
 *	Optimized for ElementSize of sizeof(void*)
 * 
 * Inputs:
 *	pArray		- Pointer to array.
 *	Index		- Index of element to get.
 *	pElement	- Pointer to storage for the element to be returned.
 * 
 * Outputs:
 *	pElement	- Copy of the desired element.
 * 
 * Returns:
 *	FSUCCESS
 *	FINVALID_PARAMETER	Index out of range.
 * 
 ****************************************************************************/
static __inline 
FSTATUS 
ArrayAtPointer(
	IN	const ARRAY* const		pArray,
	IN	const uint32			Index,
	OUT	void** const			pElement )
{
	DEBUG_ASSERT( pArray );	/* Other asserts are in ArrayGet */

	/* Range check */
	if( IB_EXPECT_FALSE(Index >= pArray->m_Size ))
		return( FINVALID_PARAMETER );

	*pElement = ArrayGetPointer( pArray, Index );
	return( FSUCCESS );
}




/****************************************************************************
 * ArraySetPointer
 * 
 * Description:
 *	This function sets the element at the specified index.  The array grows as
 *	necessary.
 *	Optimized for ElementSize of sizeof(void*)
 * 
 * Inputs:
 *	pArray		- Pointer to array.
 *	Index		- Index of element to get.
 *	Element		- the element to set at the specified index.
 *	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
 *	ppNewElements - *ppNewElements is potential new storage for array resize
 *				must have been returned by ArrayPrepareSet(Index)
 * 
 * Outputs:
 *	ppNewElements - *ppNewElements is set to NULL if ppNewElements was used
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY	The array could not be resized to accomodate the
 *							specified element.
 * 
 ****************************************************************************/
FSTATUS 
ArraySetPointer(
	IN	ARRAY* const	pArray, 
	IN	const uint32	Index,
	IN	void*			Element,
	IN	const uint32	MemFlags,
	IN OUT void**		ppNewElements OPTIONAL);

/****************************************************************************
 * ArraySetCapacity
 * 
 * Description:
 *	This function reserves capacity in the array to the specified number of
 *	elements.  This function can only fail if the new capacity is larger than
 *	the old capacity.  If the new capacity is less than the old, no action
 *	is taken.
 * 
 * Inputs:
 *	pArray		- Pointer to array
 *	NewCapacity	- Number of elements reserved in the array
 *	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
 *	ppNewElements - *ppNewElements is potential new storage for array resize
 *				must have been returned by ArrayPrepareSize(Size)
 * 
 * Outputs:
 *	ppNewElements - *ppNewElements is set to NULL if ppNewElements was used
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY - array is left unaltered
 * 
 ****************************************************************************/
FSTATUS 
ArraySetCapacity(
	IN	ARRAY* const	pArray, 
	IN	const uint32	NewCapacity,
	IN	const uint32	MemFlags,
	IN OUT void**		ppNewElements OPTIONAL);


/****************************************************************************
 * ArrayPrepareSize
 * 
 * Description:
 *	This function allocates the storage that would be needed to resize the
 *	the array to the specified number of elements.
 *	This function can only fail if the new size is larger than
 *	the old capacity.
 *
 *	This allows for premptable array allocation for arrays which must be
 *	guarded by spin locks.  For example:
 *	if (FSUCCESS != ArrayPrepareSize(&myArray, Size,
 *										IBA_MEM_FLAG_PREMPTABLE, &pNewElements))
 *		failure return
 *	SpinLockAcquire
 *	status = ArraySetSize(&myArray, Size, &pNewElements);
 *	SpinLockRelease
 *	if (pNewElements)
 *		MemoryDeallocate(pNewElements);
 * 
 * Inputs:
 *	pArray	- Pointer to array
 *	Size	- new number of elements in the the array
 *	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
 * 
 * Outputs:
 *	ppNewElements - *ppNewElements memory allocated for use in ArraySet or
 *				ArraySetSize or ArraySetCapacity
 * 
 * Returns:
 *	FSUCCESS - successfully allocated (or no allocation needed)
 *	FINSUFFICIENT_MEMORY	- unable to allocate
 *
 ****************************************************************************/
FSTATUS 
ArrayPrepareSize(
	IN	ARRAY* const	pArray, 
	IN	const uint32	Size,
	IN	const uint32	MemFlags,
	OUT void**			ppNewElements);


/****************************************************************************
 * ArrayPrepareSet
 * 
 * Description:
 *	This function allocates the storage that would be needed to resize the
 *	the array to ArraySet the specified element.
 *	This function can only fail if the Index would require a new size larger
 *	than the old capacity.
 *
 *	This allows for premptable array allocation for arrays which must be
 *	guarded by spin locks.  For example:
 *	if (FSUCCESS != ArrayPrepareSet(&myArray, Index,
 *										IBA_MEM_FLAG_PREMPTABLE, &pNewElements))
 *		failure return
 *	SpinLockAcquire
 *	status = ArraySet32(&myArray, Index, value, &pNewElements);
 *	SpinLockRelease
 *	if (pNewElements)
 *		MemoryDeallocate(pNewElements);
 * 
 * Inputs:
 *	pArray	- Pointer to array
 *	Index	- element Index which is about to be Set
 *	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
 * 
 * Outputs:
 *	ppNewElements - *ppNewElements memory allocated for use in ArraySet or
 *				ArraySetSize
 * 
 * Returns:
 *	FSUCCESS - successfully allocated (or no allocation needed)
 *	FINSUFFICIENT_MEMORY	- unable to allocate
 *
 ****************************************************************************/
static __inline FSTATUS 
ArrayPrepareSet(
	IN	ARRAY* const	pArray, 
	IN	const uint32	Index,
	IN	const uint32	MemFlags,
	OUT void**			ppNewElements)
{
	return ArrayPrepareSize(pArray, Index+1, MemFlags, ppNewElements);
}

/****************************************************************************
 * ArraySetSize
 * 
 * Description:
 *	This function resizes the array to the specified number of
 *	elements.  This function can only fail if the new size is larger than
 *	the old capacity.
 * 
 * Inputs:
 *	pArray	- Pointer to array
 *	Size	- new number of elements in the the array
 *	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
 *	ppNewElements - *ppNewElements is potential new storage for array resize
 *				must have been returned by ArrayPrepareSize(Size)
 * 
 * Outputs:
 *	ppNewElements - *ppNewElements is set to NULL if ppNewElements was used
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY	- array will have at least as many elements
 *							as prior to the call.  Use ArrayGetSize()
 *							to determine the number of elements
 * 
 ****************************************************************************/
FSTATUS 
ArraySetSize(
	IN	ARRAY* const	pArray, 
	IN	const uint32	Size,
	IN	const uint32	MemFlags,
	IN OUT void**		ppNewElements OPTIONAL);


/****************************************************************************
 * ArraySetMinSize
 * 
 * Description:
 *	This function resizes the array to the specified number of
 *	elements only if the current number of elements in the array is fewer
 *	than the specified size.
 * 
 * Inputs:
 *	pArray	- Pointer to array
 *	MinSize	- at least the number of elements needed in the array
 *	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
 *	ppNewElements - *ppNewElements is potential new storage for array resize
 *				must have been returned by ArrayPrepareSize(Size)
 * 
 * Outputs:
 *	ppNewElements - *ppNewElements is set to NULL if ppNewElements was used
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY - array is left unaltered
 * 
 ****************************************************************************/
FSTATUS ArraySetMinSize(
	IN	ARRAY* const	pArray, 
	IN	const uint32	MinSize,
	IN	const uint32	MemFlags,
	IN OUT void**		ppNewElements OPTIONAL);


/****************************************************************************
 * ArrayApplyFunc
 * 
 * Description:
 *	This function calls the user supplied function for each item in the array,
 *	starting from the beginning of the array.
 * 
 *	The called function has the form:
 *		void ApplyFunc( uint32 Index, void *pElement, void *Context );
 *	where:
 *		Index = index of this element
 *		pElement = void* at this location in the array
 *		Context = user supplied context
 * 
 * Inputs:
 *	pArray		- pointer to array to iterate through
 *	pfnCallback	- callback called for each non-NULL element
 *	Context		- context to pass to callback function
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void 
ArrayApplyFunc(
	IN	const ARRAY* const	pArray, 
	IN	ARR_APPLY_FUNC		pfnCallback,
	IN	void* const			Context );


/****************************************************************************
 * ArrayFindFromStart
 * 
 * Description:
 *	This function calls the specified function for each item in the array, 
 *	starting from the beginning of the array, until the callback function 
 *	returns TRUE or every item was processed.  
 * 
 *	The called function has the form:
 *		boolean FindFunc( uint32 Index, void *pElement, void *Context );
 *	where:
 *		Index = index of this element
 *		pElement = pointer to the element at this location in the array
 *		Context = user supplied context
 * 
 * Inputs:
 *	pArray		- pointer to array to iterate through
 *	pfnCallback	- callback called for each non-NULL element
 *	Context		- context to pass to callback function
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	Index value where execution stopped.  If the callback function never
 *	returned TRUE, then the return value = the size of the array.
 * 
 ****************************************************************************/
uint32 
ArrayFindFromStart(
	IN	const ARRAY* const	pArray, 
	IN	ARR_FIND_FUNC		pfnCallback,
	IN	void* const			Context );


/****************************************************************************
 * ArrayFindFromEnd
 * 
 * Description:
 *	This function calls the specified function for each item in the array, 
 *	starting from the end of the array, until the callback function returns 
 *	TRUE or every item was processed.  
 * 
 *	The called function has the form:
 *		boolean FindFunc( uint32 Index, void *pElement, void *Context );
 *	where:
 *		Index = index of this element
 *		pElement = pointer to the element at this location in the array
 *		Context = user supplied context
 * 
 * Inputs:
 *	pArray		- pointer to array to iterate through
 *	pfnCallback	- callback called for each non-NULL element
 *	Context		- context to pass to callback function
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	Index value where execution stopped.  If the callback function never
 *	returned TRUE, then the return value = the size of the array.
 * 
 ****************************************************************************/
uint32 
ArrayFindFromEnd(
	IN	const ARRAY* const	pArray, 
	IN	ARR_FIND_FUNC		pfnCallback,
	IN	void* const			Context );


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
 );

#ifdef __cplusplus
}	/* extern "C" */
#endif


#endif	/* _IBA_PUBLIC_IARRAY_H_ */
