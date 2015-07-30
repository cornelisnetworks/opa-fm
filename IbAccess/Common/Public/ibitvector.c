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


#include "ibitvector.h"
#include "imemory.h"
#include "imath.h"

// Make a memory tag of 'ivec' for ivector.
#define VEC_MEM_TAG		MAKE_MEM_TAG( i, v, e, c )


///////////////////////////////////////////////////////////////////////////////
// bit vector access methods
// These methods are an attempt to follow the functionality of the C++ STL.
// The names and behavior are obviously not exact matches, but hopefully,
// close enough to make those familiar with the ANSI standard comfortable.
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// BitVectorInitState
// 
// Description:
//	This function performs basic bit vector initialization.  This function cannot 
//	fail.
// 
// Inputs:
//	pBitVector - Pointer to bit vector
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void 
BitVectorInitState(
	IN	BIT_VECTOR* const	pBitVector )
{
	ASSERT( pBitVector );

	MemoryClear( pBitVector, sizeof( BIT_VECTOR ) );
}



///////////////////////////////////////////////////////////////////////////////
// BitVectorInit
// 
// Description:
//	This function initializes the bit vector.  New bits in the bit vector are
//	initialized to 0.
// 
// Inputs:
//	pBitVector		- Pointer to bit vector
//	InitialSize	- prefered initial number of bits
//	GrowSize	- number of bits to allocate when incrementally growing
//				the bit vector (will round up to multiple of processor
//				word size)
//	MaxSize		- maximum number of bits in bit vector, 0 -> no limit
//	Context		- Context value passed to the callbacks.
//	IsPageable	- TRUE indicates the bit vector should use pageable memory
//				FALSE indicates the bit vector should used non-paged memory
// 
// Outputs:
//	None.
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY - bit vector is initialized to zero size
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS
BitVectorInit(
	IN	BIT_VECTOR* const	pBitVector,
	IN	const uint32	InitialSize,
	IN	const uint32	GrowSize,
	IN	const uint32	MaxSize,
	IN	boolean			IsPageable )
{
	FSTATUS	Status;

	ASSERT( pBitVector );
	ASSERT( GrowSize );

	BitVectorInitState( pBitVector );

	pBitVector->m_GrowSize = ROUNDUPP2(GrowSize, _BIT_VECTOR_ENTRY_SIZE);
	pBitVector->m_MaxSize = (MaxSize == 0)? IB_UINT32_MAX : MaxSize;
	pBitVector->m_Pageable = IsPageable;

	// get the storage needed by the user
	Status = BitVectorSetSize( pBitVector, MIN(InitialSize, pBitVector->m_MaxSize) );

	return( Status );
}


///////////////////////////////////////////////////////////////////////////////
// BitVectorDestroy
// 
// Description:
//	This function deallocates all allocated memory associated with the bit vector.
//	The bit vector is left initialized to zero size.  This function does not fail.
// 
// Inputs:
//	pBitVector - Pointer to bit vector
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void
BitVectorDestroy(
	IN	BIT_VECTOR* const	pBitVector )
{
	ASSERT( pBitVector );

	// Destroy the page bit vector.
	if( pBitVector->m_pBitArray )
	{
		MemoryDeallocate( pBitVector->m_pBitArray );
	}

	// reset internal state
	pBitVector->m_Capacity = 0;
	pBitVector->m_GrowSize = 0;
	pBitVector->m_Size = 0;
	pBitVector->m_pBitArray = NULL;

	return;
}


///////////////////////////////////////////////////////////////////////////////
// BitVectorAt
// 
// Description:
//	This function returns the bit at the specified index.
//	Range checking is performed.  Provides constant time access.
// 
// Inputs:
//	pBitVector		- Pointer to bit vector.
//	Index		- Index of bit to get.
//	pValue	- Pointer to storage for the bit to be returned.
// 
// Outputs:
//	pValue	- Copy of the desired bit.
// 
// Returns:
//	FSUCCESS
//	FINVALID_SETTING	Index out of range.
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS
BitVectorAt(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	const uint32		Index,
	OUT	uint8*				pValue )
{
	ASSERT( pBitVector );	// Other asserts are in BitVectorGet

	// Range check
	if( Index >= pBitVector->m_Size )
		return( FINVALID_PARAMETER );

	*pValue = BitVectorGet( pBitVector, Index );
	return( FSUCCESS );
}


///////////////////////////////////////////////////////////////////////////////
// BitVectorSet
// 
// Description:
//	This function sets the bit at the specified index.  The array grows as
//	necessary.
// 
// Inputs:
//	pBitVector		- Pointer to bit vector.
//	Index		- Index of bit to get.
//	Value		- value to set the bit to
// 
// Outputs:
//	None.
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY	The array could not be resized to accomodate the
//							specified bit.
//	FINSUFFICIENT_RESOURCES	- would grow vector beyond max size
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
BitVectorSet(
	IN	BIT_VECTOR* const	pBitVector, 
	IN	const uint32	Index,
	IN	const uint8		Value )
{
	FSTATUS	Status;

	ASSERT( pBitVector );

	// Determine if the bit vector has room for this bit.
	if( Index >= pBitVector->m_Size )
	{
		// Resize to accomodate the given index.
		Status = BitVectorSetSize( pBitVector, Index + 1 );

		// Check for failure on or before the given index.
		if( (Status != FSUCCESS) && (pBitVector->m_Size <= Index) )
			return( Status );
		if (pBitVector->m_Size <= Index) {
			return( FINSUFFICIENT_MEMORY );
		}
	}

	// At this point, the array is guaranteed to be big enough
	if (Value)
	{
		pBitVector->m_pBitArray[_BitVectorSubscript(Index)] |= _BitVectorMask(Index);
	} else {
		pBitVector->m_pBitArray[_BitVectorSubscript(Index)] &= ~_BitVectorMask(Index);
	}

	return( FSUCCESS );
}


///////////////////////////////////////////////////////////////////////////////
// BitVectorSetCapacity
// 
// Description:
//	This function reserves capacity in the bit vector to the specified number of
//	bits.  This function can only fail if the new capacity is larger than
//	the old capacity.  If the new capacity is less than the old, no action
//	is taken.  When growing the capacity, new bits will have value 0.
// 
// Inputs:
//	pBitVector		- Pointer to bit vector
//	NewCapacity	- Number of bits reserved in the array
// 
// Outputs:
//	None.
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY - bit vector is left unaltered
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
BitVectorSetCapacity(
	IN	BIT_VECTOR* const	pBitVector, 
	IN	uint32				NewCapacity )
{
	unsigned	*pNewBitArray;

	ASSERT( pBitVector );

	if (NewCapacity > pBitVector->m_MaxSize)
	{
		NewCapacity = pBitVector->m_MaxSize;
	}

	// Do we have to do anything here?
	if( NewCapacity <= pBitVector->m_Capacity )
	{
		// Nope
		return( FSUCCESS );
	}

	// Allocate our bit array.
	pNewBitArray = (unsigned*)MemoryAllocateAndClear( ((NewCapacity+_BIT_VECTOR_ENTRY_SIZE)/_BIT_VECTOR_ENTRY_SIZE) * sizeof( unsigned ), 
		pBitVector->m_Pageable, VEC_MEM_TAG );
	if( !pNewBitArray )
		return( FINSUFFICIENT_MEMORY );

	if( pBitVector->m_pBitArray )
	{
		// Copy the old bit array into the new.
		MemoryCopy( pNewBitArray, pBitVector->m_pBitArray, 
			((pBitVector->m_Capacity+_BIT_VECTOR_ENTRY_SIZE)/_BIT_VECTOR_ENTRY_SIZE) * sizeof( unsigned ) );

		// Free the old bit array.
		MemoryDeallocate( pBitVector->m_pBitArray );
	}
	
	// Set the new array.
	pBitVector->m_pBitArray = pNewBitArray;

	// Update the bit vector with the new capacity.
	pBitVector->m_Capacity = NewCapacity;

	return( FSUCCESS );
}


///////////////////////////////////////////////////////////////////////////////
// BitVectorSetSize
// 
// Description:
//	This function resizes the bit vector to the specified number of
//	bits.  This function can only fail if the new size is larger than
//	the old capacity.
// 
// Inputs:
//	pBitVector	- Pointer to bit vector
//	Size	- new number of bits in the the array
// 
// Outputs:
//	None.
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY	- bit vector will have at least as many bits
//							as prior to the call.  Use BitVectorGetSize()
//							to determine the number of bits
//	FINSUFFICIENT_RESOURCES	- would grow vector beyond max size
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
BitVectorSetSize(
	IN	BIT_VECTOR* const	pBitVector, 
	IN	const uint32	Size )
{
	FSTATUS	Status;
	uint32	NewCapacity;

	ASSERT( pBitVector );

	if( Size == pBitVector->m_Size )
		return( FSUCCESS );	// no change
	if (Size > pBitVector->m_MaxSize)
		return( FINSUFFICIENT_RESOURCES );

	// Determine if the bit vector has room.
	if( Size >= pBitVector->m_Capacity )
	{
		// keep capacity as a multiple of GrowSize
		NewCapacity = ROUNDUP(Size, pBitVector->m_GrowSize);
		Status = BitVectorSetCapacity( pBitVector, NewCapacity );
		if( Status != FSUCCESS )
			return( Status );
	}

	pBitVector->m_Size = Size;
	return( FSUCCESS );
}


///////////////////////////////////////////////////////////////////////////////
// BitVectorSetMinSize
// 
// Description:
//	This function resizes the bit vector to the specified number of
//	bits only if the current number of bits in the array is fewer
//	than the specified size.
// 
// Inputs:
//	pBitVector	- Pointer to bit vector
//	MinSize	- at least the number of bitbits needed in the array
// 
// Outputs:
//	None.
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY - bit vector is left unaltered
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS BitVectorSetMinSize(
	IN	BIT_VECTOR* const	pBitVector, 
	IN	const uint32	MinSize )
{
	ASSERT( pBitVector );

	if( MinSize > pBitVector->m_Size )
	{
		// We have to resize the array
		return( BitVectorSetSize( pBitVector, MinSize ) );
	}

	// We didn't have to do anything
	return( FSUCCESS );
}


///////////////////////////////////////////////////////////////////////////////
// BitVectorApplyFunc
// 
// Description:
//	This function calls the user supplied function for each item in the bit vector.
// 
//	The called function has the form:
//		void ApplyFunc( uint32 Index, uint8 Value, void *Context );
//	where:
//		Index = index of this bit
//		Value = value of bit at this Index in the array
//		Context = user supplied context
// 
// Inputs:
//	pBitVector		- pointer to bit vector to iterate through
//	pfnCallback	- callback called for each non-NULL bit
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
BitVectorApplyFunc(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	BITVEC_APPLY_FUNC	pfnCallback,
	IN	void* const			Context )
{
	return BitVectorApplyFuncRange(pBitVector, pfnCallback, Context,
									0, pBitVector->m_Size);
}


///////////////////////////////////////////////////////////////////////////////
// BitVectorApplyFuncRange
// 
// Description:
//	This function calls the user supplied function for each item in the bit vector
//	across the given range of indexes of the bit vector.
// 
//	The called function has the form:
//		void ApplyFunc( uint32 Index, uint8 Value, void *Context );
//	where:
//		Index = index of this bit
//		Value = value of bit at this Index in the array
//		Context = user supplied context
// 
// Inputs:
//	pBitVector		- pointer to bit vector to iterate through
//	pfnCallback	- callback called for each non-NULL bit
//	Context		- context to pass to callback function
//	StartIndex		- Index to start at
//	EndIndexP1		- Index to stop at (exclusive)
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void 
BitVectorApplyFuncRange(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	BITVEC_APPLY_FUNC	pfnCallback,
	IN	void* const			Context,
	IN	uint32				StartIndex,
	IN	uint32				EndIndexP1 )
{
	uint32	i;
	uint8 Value;

	ASSERT( pBitVector );
	ASSERT( pfnCallback );

	i  = StartIndex;
	if (EndIndexP1 > pBitVector->m_Size)
		EndIndexP1 = pBitVector->m_Size;
	for( ; i < EndIndexP1; ++i )
	{
		Value = BitVectorGet( pBitVector, i );
		pfnCallback( i, Value, Context );
	}
}


///////////////////////////////////////////////////////////////////////////////
// BitVectorApplyFuncSelected
// 
// Description:
//	This function calls the user supplied function for each bit in the bit vector,
//	which matches Value, starting from the beginning of the array.
// 
//	The called function has the form:
//		void ApplyFunc( uint32 Index, uint8 Value, void *Context );
//	where:
//		Index = index of this bit
//		Value = value of bit at this Index in the array
//		Context = user supplied context
// 
// Inputs:
//	pBitVector		- pointer to bit vector to iterate through
//	pfnCallback	- callback called for each non-NULL bit
//	Context		- context to pass to callback function
//	Value		- Bit values to execute pfnCallback for
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void 
BitVectorApplyFuncSelected(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	BITVEC_APPLY_FUNC	pfnCallback,
	IN	void* const			Context,
	IN	uint8				Value )
{
	return BitVectorApplyFuncSelectedRange(pBitVector, pfnCallback, Context,
						Value, 0, pBitVector->m_Size);
}

///////////////////////////////////////////////////////////////////////////////
// BitVectorApplyFuncSelectedRange
// 
// Description:
//	This function calls the user supplied function for each bit in the bit vector,
//	across the given range of indexes of the bit vector,
//	which matches Value, starting from the beginning of the array.
// 
//	The called function has the form:
//		void ApplyFunc( uint32 Index, uint8 Value, void *Context );
//	where:
//		Index = index of this bit
//		Value = value of bit at this Index in the array
//		Context = user supplied context
// 
// Inputs:
//	pBitVector		- pointer to bit vector to iterate through
//	pfnCallback	- callback called for each non-NULL bit
//	Context		- context to pass to callback function
//	Value		- Bit values to execute pfnCallback for
//	StartIndex		- Index to start at
//	EndIndexP1		- Index to stop at (exclusive)
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void 
BitVectorApplyFuncSelectedRange(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	BITVEC_APPLY_FUNC	pfnCallback,
	IN	void* const			Context,
	IN	uint8				Value,
	IN	uint32				StartIndex,
	IN	uint32				EndIndexP1 )
{
	uint32	i;
	uint8 Val;

	ASSERT( pBitVector );
	ASSERT( pfnCallback );

	i  = StartIndex;
	if (EndIndexP1 > pBitVector->m_Size)
		EndIndexP1 = pBitVector->m_Size;
	for( ; i < EndIndexP1; ++i )
	{
		Val = BitVectorGet( pBitVector, i );
		if (Value == Val)
			pfnCallback( i, Value, Context );
	}
}

///////////////////////////////////////////////////////////////////////////////
// BitVectorFindFromStart
// 
// Description:
//	This function searches the Bit Vector for the given value
//	starting from the beginning of the bit vector, until a bit matching
//	Value is found
// 
// Inputs:
//	pBitVector		- pointer to bit vector to iterate through
//	Value			- bit value to find
// 
// Outputs:
//	None.
// 
// Returns:
//	Index value where the first bit with the given value was found.
//	If the callback given value was never found,
//	then the return value = the size of the bit vector.
// 
///////////////////////////////////////////////////////////////////////////////
uint32 
BitVectorFindFromStart(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	uint8				Value )
{
	return BitVectorFindRange(pBitVector, Value, 0, pBitVector->m_Size);
}

///////////////////////////////////////////////////////////////////////////////
// BitVectorFindFromIndex
// 
// Description:
//	This function searches the Bit Vector for the given value
//	starting from the given index of the bit vector, until a bit matching
//	Value is found.  Search proceeds forward (increasing indexes).
// 
// Inputs:
//	pBitVector		- pointer to bit vector to iterate through
//	Value			- bit value to find
//	Index			- Index to start at
// 
// Outputs:
//	None.
// 
// Returns:
//	Index value where the first bit with the given value was found.
//	If the given value was never found,
//	then the return value = the size of the bit vector.
// 
///////////////////////////////////////////////////////////////////////////////
uint32 
BitVectorFindFromIndex(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	uint8				Value,
	IN	uint32				Index )
{
	return BitVectorFindRange(pBitVector, Value, Index, pBitVector->m_Size);
}

///////////////////////////////////////////////////////////////////////////////
// BitVectorFindRange
// 
// Description:
//	This function searches the Bit Vector for the given value
//	across the given range of indexes of the bit vector, until a bit matching
//	Value is found.  Search proceeds forward (increasing indexes).
// 
// Inputs:
//	pBitVector		- pointer to bit vector to iterate through
//	Value			- bit value to find
//	StartIndex		- Index to start at
//	EndIndexP1		- Index to stop at (exclusive)
// 
// Outputs:
//	None.
// 
// Returns:
//	Index value where the first bit with the given value was found.
//	If the given value was never found,
//	then the return value = EndIndexP1
// 
///////////////////////////////////////////////////////////////////////////////
uint32 
BitVectorFindRange(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	uint8				Value,
	IN	uint32				StartIndex,
	IN	uint32				EndIndexP1 )
{
	uint32	i;
	uint32 *pEntry;
	uint32 mask;
	uint32 inc;

	ASSERT( pBitVector );

	// for speed we unroll the Value test and do 32 bits at a time
	i  = StartIndex;
	if (EndIndexP1 > pBitVector->m_Size)
		EndIndexP1 = pBitVector->m_Size;
	pEntry = &pBitVector->m_pBitArray[_BitVectorSubscript(i)];
	// First word is special case with smaller mask and increment
	inc = _BIT_VECTOR_ENTRY_SIZE-_BitVectorMaskShift(i);
	mask = ~0<<_BitVectorMaskShift(i);
	if (Value)
	{
		while(i < EndIndexP1)
		{
			// expect optimizes the looping case
			if (IB_EXPECT_FALSE((*pEntry & mask) != 0))
			{
				// this is the entry with our bit
				while(i < EndIndexP1)
				{
					if (*pEntry & _BitVectorMask(i))
						goto done;
					i++;
				}
				goto done;
			}
			++pEntry;
			i+=inc;
			inc = _BIT_VECTOR_ENTRY_SIZE;
			mask = ~0;
		}
	} else {
		while(i < EndIndexP1)
		{
			// expect optimizes the looping case
			if (IB_EXPECT_FALSE((*pEntry & mask) != mask))
			{
				// this is the entry with our bit
				while(i < EndIndexP1)
				{
					if (0 == (*pEntry & _BitVectorMask(i)))
						goto done;
					i++;
				}
				goto done;
			}
			++pEntry;
			i+=inc;
			inc = _BIT_VECTOR_ENTRY_SIZE;
			mask = ~0;
		}
	}
done:
	// In case EndIndexP1 is not a multiple of 32, and we over incremented in the outer loop
	if (i > EndIndexP1) i = EndIndexP1;

	DEBUG_ASSERT((EndIndexP1 == i) || (Value && BitVectorGet(pBitVector, i)) || (!Value && !BitVectorGet(pBitVector, i)));
	return( i );
}


///////////////////////////////////////////////////////////////////////////////
// BitVectorFindFromEnd
// 
// Description:
//	This function searches the Bit Vector for the given value
//	starting from the end of the bit vector, until a bit matching
//	Value is found
// 
// Inputs:
//	pBitVector		- pointer to bit vector to iterate through
//	Value			- bit value to find
// 
// Outputs:
//	None.
// 
// Returns:
//	Index value where the first bit with the given value was found.
//	If the callback given value was never found,
//	then the return value = the size of the bit vector.
// 
///////////////////////////////////////////////////////////////////////////////
uint32 
BitVectorFindFromEnd(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	uint8				Value )
{
	uint32	i;
	uint32 *pEntry;

	ASSERT( pBitVector );

	// for speed we unroll the Value test and do 32 bits at a time
	i  = pBitVector->m_Size;
	pEntry = &pBitVector->m_pBitArray[_BitVectorSubscript(i)];
	if (Value)
	{
		for( ; i; i-=32, --pEntry)
		{
			// expect optimizes the looping case
			if (IB_EXPECT_FALSE(*pEntry))
			{
				// this is the entry with our bit
				for (; i; --i)
				{
					if (*pEntry & _BitVectorMask(i))
						goto found;
				}
				goto notfound;
			}
		}
	} else {
		for( ; i; i-=32, --pEntry)
		{
			// expect optimizes the looping case
			if (IB_EXPECT_FALSE(*pEntry != 0xffffffff))
			{
				// this is the entry with our bit
				for (; i; --i)
				{
					if (0 == (*pEntry & _BitVectorMask(i)))
						goto found;
				}
				goto notfound;
			}
		}
	}
notfound:
	return (pBitVector->m_Size);

found:
	return( i );
}
