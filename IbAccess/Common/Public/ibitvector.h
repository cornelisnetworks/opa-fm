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


#ifndef _IBA_PUBLIC_IBITVECTOR_H_
#define _IBA_PUBLIC_IBITVECTOR_H_

#include "iba/public/datatypes.h"


#ifdef __cplusplus
extern "C"
{
#endif

/* Callback function declarations. */
typedef void (*BITVEC_APPLY_FUNC)( 
	IN const uint32 Index, 
	IN uint8 const Value,	/* 0 or 1 */
	IN void* const Context );

/****************************************************************************
 *
 * BitVector definition.
 *
 * BitVector statically stores items as bits in an array, providing constant
 * access times.  The array can be resized dynamically.
 */


/* The bit vector structure is allocated by the user and is opaque to them */
typedef struct _BIT_VECTOR
{ 
	/* all fields below are private, use accessor functions to */
	/* manipulate this structure */
	uint32			m_Size;		/* # of bits logically in the bit vector. */
	uint32			m_GrowSize;	/* # of bits to grab when growing. */
	uint32			m_MaxSize;	/* Max # bits to grow to, 0-> no limit */
	uint32			m_Capacity;	/* total # of bits allocated. */
	boolean			m_Pageable;	/* User supplied pageable memory flag. */
	unsigned 		*m_pBitArray;	/* Internal vector of bits */
} BIT_VECTOR;

#define _BIT_VECTOR_ENTRY_SIZE (sizeof(unsigned)*8)	/* bits per Array entry */

/* some private inline functions */
static __inline uint32
_BitVectorSubscript(uint32 bit)
{
	return (bit / _BIT_VECTOR_ENTRY_SIZE);
}

static __inline uint32
_BitVectorMaskShift(uint32 bit)
{
	return (bit& (_BIT_VECTOR_ENTRY_SIZE-1));
}

static __inline uint32
_BitVectorMask(uint32 bit)
{
	return (1<< _BitVectorMaskShift(bit));
}

static __inline uint32
_BitVectorBit(uint32 subscript, uint32 mask_shift)
{
	return ((subscript*_BIT_VECTOR_ENTRY_SIZE)+mask_shift);
}


/****************************************************************************
 * bit vector access methods
 * These methods are an attempt to follow the functionality of the C++ STL.
 * The names and behavior are obviously not exact matches, but hopefully,
 * close enough to make those familiar with the ANSI standard comfortable.
 ****************************************************************************/


/****************************************************************************
 * BitVectorInitState
 * 
 * Description:
 *	This function performs basic bit vector initialization.
 *	This function cannot fail.
 * 
 * Inputs:
 *	pBitVector - Pointer to bit vector
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void 
BitVectorInitState(
	IN	BIT_VECTOR* const	pBitVector );


/****************************************************************************
 * BitVectorInit
 * 
 * Description:
 *	This function initializes the bit vector.  New bits in the bit vector are
 *	initialized to 0.
 * 
 * Inputs:
 *	pBitVector		- Pointer to bit bit vector
 *	InitialSize	- prefered initial number of bits
 *	GrowSize	- number of bits to allocate when incrementally growing
 *				the bit vector
 *	MaxSize		- maximum number of bits in bit vector, 0 -> no limit
 *	IsPageable	- TRUE indicates the bit vector should use pageable memory
 *				FALSE indicates the bit vector should used non-paged memory
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY - bit vector is initialized to zero size
 * 
 ****************************************************************************/
FSTATUS
BitVectorInit(
	IN	BIT_VECTOR* const	pBitVector,
	IN	const uint32	InitialSize,
	IN	const uint32	GrowSize,
	IN	const uint32	MaxSize,
	IN	boolean			IsPageable );


/****************************************************************************
 * BitVectorDestroy
 * 
 * Description:
 *	This function deallocates all allocated memory associated with the bit
 *	vector.
 *	The bit vector is left initialized to zero size.
 *	This function does not fail.
 * 
 * Inputs:
 *	pBitVector - Pointer to bit vector
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void
BitVectorDestroy(
	IN	BIT_VECTOR* const	pBitVector );


/****************************************************************************
 * BitVectorGetCapacity
 *
 * Description:
 *	This function returns the current capacity of the bit vector. 
 *	This function cannot fail.
 *
 * Inputs:
 *	pBitVector	- Pointer to bit vector
 *
 * Outputs:
 *	None.
 *
 * Returns:
 *	Capacity, in bits, of the bit vector.
 *
 ****************************************************************************/
static __inline uint32
BitVectorGetCapacity(
	IN	const BIT_VECTOR* const	pBitVector )
{
	DEBUG_ASSERT( pBitVector );
	return( pBitVector->m_Capacity );
}


/****************************************************************************
 * BitVectorGetSize
 *
 * Description:
 *	This function returns the size of the bit vector. This function cannot fail.
 *
 * Inputs:
 *	pBitVector	- Pointer to bit vector
 *
 * Outputs:
 *	None.
 *
 * Returns:
 *	Size, in bits, of the bit vector.
 *
 ****************************************************************************/
static __inline uint32
BitVectorGetSize(
	IN	const BIT_VECTOR* const	pBitVector )
{
	DEBUG_ASSERT( pBitVector );
	return( pBitVector->m_Size );
}


/****************************************************************************
 * BitVectorGetMaxSize
 *
 * Description:
 *	This function returns the max size of the bit vector.
 *	This function cannot fail.
 *
 * Inputs:
 *	pBitVector	- Pointer to bit vector
 *
 * Outputs:
 *	None.
 *
 * Returns:
 *	MaxSize, in bits, of the bit vector. (0-> no limit)
 *
 ****************************************************************************/
static __inline uint32
BitVectorGetMaxSize(
	IN	const BIT_VECTOR* const	pBitVector )
{
	DEBUG_ASSERT( pBitVector );
	return( pBitVector->m_MaxSize );
}

/****************************************************************************
 * BitVectorGet
 * 
 * Description:
 *	This function returns the value of the bit at the specified index.
 *	Range checking is NOT performed.  Provides constant time access.
 * 
 * Inputs:
 *	pBitVector	- Pointer to bit vector.
 *	Index	- Index of bit to get.
 * 
 * Outputs:
 *	value of bit
 * 
 * Returns:
 *	None.
 * 
 * Notes:
 * 
 ****************************************************************************/
static __inline uint8
BitVectorGet(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	const uint32		Index )
{
	DEBUG_ASSERT(pBitVector);
	return( (pBitVector->m_pBitArray[_BitVectorSubscript(Index)]&_BitVectorMask(Index)) >> _BitVectorMaskShift(Index) );
}


/****************************************************************************
 * BitVectorAt
 * 
 * Description:
 *	This function returns the bit at the specified index.
 *	Range checking is performed.  Provides constant time access.
 * 
 * Inputs:
 *	pBitVector		- Pointer to bit vector.
 *	Index		- Index of element to get.
 *	pValue	- Pointer to storage for the bit to be returned.
 * 
 * Outputs:
 *	pValue	- value of given bit
 * 
 * Returns:
 *	FSUCCESS
 *	FINVALID_SETTING	Index out of range.
 * 
 ****************************************************************************/
FSTATUS
BitVectorAt(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	const uint32		Index,
	OUT	uint8*				pValue );

/****************************************************************************
 * BitVectorSet
 * 
 * Description:
 *	This function sets the bit at the specified index to the given value.
 *	The array grows as necessary.
 * 
 * Inputs:
 *	pBitVector		- Pointer to bit vector.
 *	Index		- Index of bit to set to value.
 *	Value		- value to set the bit to
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY	The array could not be resized to accomodate the
 *							specified bit.
 *	FINSUFFICIENT_RESOURCES	- would grow vector beyond max size
 * 
 ****************************************************************************/
FSTATUS 
BitVectorSet(
	IN	BIT_VECTOR* const	pBitVector, 
	IN	const uint32	Index,
	IN	const uint8 	Value );


/****************************************************************************
 * BitVectorSetCapacity
 * 
 * Description:
 *	This function reserves capacity in the bit vector to the specified number of
 *	bit.  This function can only fail if the new capacity is larger than
 *	the old capacity.  If the new capacity is less than the old, no action
 *	is taken.  When growing the capacity, new bits will have value 0.
 * 
 * Inputs:
 *	pBitVector		- Pointer to bit vector
 *	NewCapacity	- Number of bit reserved in the array
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY - bit vector is left unaltered
 * 
 ****************************************************************************/
FSTATUS 
BitVectorSetCapacity(
	IN	BIT_VECTOR* const	pBitVector, 
	IN	uint32				NewCapacity );


/****************************************************************************
 * BitVectorSetSize
 * 
 * Description:
 *	This function resizes the bit vector to the specified number of
 *	bit.  This function can only fail if the new size is larger than
 *	the old capacity.
 * 
 * Inputs:
 *	pBitVector	- Pointer to bit vector
 *	Size	- new number of bit in the the array
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY	- bit vector will have at least as many bit
 *							as prior to the call.  Use BitVectorGetSize()
 *							to determine the number of bit
 *	FINSUFFICIENT_RESOURCES	- would grow vector beyond max size
 * 
 ****************************************************************************/
FSTATUS 
BitVectorSetSize(
	IN	BIT_VECTOR* const	pBitVector, 
	IN	const uint32	Size );


/****************************************************************************
 * BitVectorSetMinSize
 * 
 * Description:
 *	This function resizes the bit vector to the specified number of
 *	bit only if the current number of bit in the array is fewer
 *	than the specified size.
 * 
 * Inputs:
 *	pBitVector	- Pointer to bit vector
 *	MinSize	- at least the number of bit needed in the array
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY - bit vector is left unaltered
 * 
 ****************************************************************************/
FSTATUS BitVectorSetMinSize(
	IN	BIT_VECTOR* const	pBitVector, 
	IN	const uint32	MinSize );


/****************************************************************************
 * BitVectorApplyFunc
 * 
 * Description:
 *	This function calls the user supplied function for each bit in the bit vector,
 *	starting from the beginning of the array.
 * 
 *	The called function has the form:
 *		void ApplyFunc( uint32 Index, uint8 Value, void *Context );
 *	where:
 *		Index = index of this bit
 *		Value = value of bit at this Index in the array
 *		Context = user supplied context
 * 
 * Inputs:
 *	pBitVector		- pointer to bit vector to iterate through
 *	pfnCallback	- callback called for each non-NULL bit
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
BitVectorApplyFunc(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	BITVEC_APPLY_FUNC	pfnCallback,
	IN	void* const			Context );


/****************************************************************************
 * BitVectorApplyFuncRange
 * 
 * Description:
 *	This function calls the user supplied function for each item in the bit
 *	vector across the given range of indexes of the bit vector.
 * 
 *	The called function has the form:
 *		void ApplyFunc( uint32 Index, uint8 Value, void *Context );
 *	where:
 *		Index = index of this bit
 *		Value = value of bit at this Index in the array
 *		Context = user supplied context
 * 
 * Inputs:
 *	pBitVector		- pointer to bit vector to iterate through
 *	pfnCallback	- callback called for each non-NULL bit
 *	Context		- context to pass to callback function
 *	StartIndex		- Index to start at
 *	EndIndexP1		- Index to stop at (exclusive)
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void 
BitVectorApplyFuncRange(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	BITVEC_APPLY_FUNC	pfnCallback,
	IN	void* const			Context,
	IN	uint32				StartIndex,
	IN	uint32				EndIndexP1 );

/****************************************************************************
 * BitVectorApplyFuncSelected
 * 
 * Description:
 *	This function calls the user supplied function for each bit in the bit
 *	vector,	which matches Value, starting from the beginning of the array.
 * 
 *	The called function has the form:
 *		void ApplyFunc( uint32 Index, uint8 Value, void *Context );
 *	where:
 *		Index = index of this bit
 *		Value = value of bit at this Index in the array
 *		Context = user supplied context
 * 
 * Inputs:
 *	pBitVector		- pointer to bit vector to iterate through
 *	pfnCallback	- callback called for each non-NULL bit
 *	Context		- context to pass to callback function
 *	Value		- Bit values to execute pfnCallback for
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void 
BitVectorApplyFuncSelected(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	BITVEC_APPLY_FUNC	pfnCallback,
	IN	void* const			Context,
	IN	uint8				Value );


/****************************************************************************
 * BitVectorApplyFuncSelectedRange
 * 
 * Description:
 *	This function calls the user supplied function for each bit in the bit
 *	vector,	across the given range of indexes of the bit vector,
 *	which matches Value, starting from the beginning of the array.
 * 
 *	The called function has the form:
 *		void ApplyFunc( uint32 Index, uint8 Value, void *Context );
 *	where:
 *		Index = index of this bit
 *		Value = value of bit at this Index in the array
 *		Context = user supplied context
 * 
 * Inputs:
 *	pBitVector		- pointer to bit vector to iterate through
 *	pfnCallback	- callback called for each non-NULL bit
 *	Context		- context to pass to callback function
 *	Value		- Bit values to execute pfnCallback for
 *	StartIndex		- Index to start at
 *	EndIndexP1		- Index to stop at (exclusive)
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void 
BitVectorApplyFuncSelectedRange(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	BITVEC_APPLY_FUNC	pfnCallback,
	IN	void* const			Context,
	IN	uint8				Value,
	IN	uint32				StartIndex,
	IN	uint32				EndIndexP1 );

/****************************************************************************
 * BitVectorFindFromStart
 * 
 * Description:
 *	This function searches the Bit Vector for the given value
 *	starting from the beginning of the bit vector, until a bit matching
 *	Value is found
 * 
 * Inputs:
 *	pBitVector		- pointer to bit vector to iterate through
 *	Value			- bit value to find
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	Index value where the first bit with the given value was found.
 *	If the callback given value was never found,
 *	then the return value = the size of the bit vector.
 * 
 ****************************************************************************/
uint32 
BitVectorFindFromStart(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	uint8				Value );

/****************************************************************************
 * BitVectorFindFromIndex
 * 
 * Description:
 *	This function searches the Bit Vector for the given value
 *	starting from the given index of the bit vector, until a bit matching
 *	Value is found.  Search proceeds forward (increasing indexes).
 * 
 * Inputs:
 *	pBitVector		- pointer to bit vector to iterate through
 *	Value			- bit value to find
 *	Index			- Index to start at
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	Index value where the first bit with the given value was found.
 *	If the callback given value was never found,
 *	then the return value = the size of the bit vector.
 * 
 ****************************************************************************/
uint32 
BitVectorFindFromIndex(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	uint8				Value,
	IN	uint32				Index );

/****************************************************************************
 * BitVectorFindRange
 * 
 * Description:
 *	This function searches the Bit Vector for the given value
 *	across the given range of indexes of the bit vector, until a bit matching
 *	Value is found.  Search proceeds forward (increasing indexes).
 * 
 * Inputs:
 *	pBitVector		- pointer to bit vector to iterate through
 *	Value			- bit value to find
 *	StartIndex		- Index to start at
 *	EndIndexP1		- Index to stop at (exclusive)
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	Index value where the first bit with the given value was found.
 *	If the given value was never found,
 *	then the return value = EndIndexP1
 * 
 ****************************************************************************/
uint32 
BitVectorFindRange(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	uint8				Value,
	IN	uint32				StartIndex,
	IN	uint32				EndIndexP1);


/****************************************************************************
 * BitVectorFindFromEnd
 * 
 * Description:
 *	This function searches the Bit Vector for the given value
 *	starting from the end of the bit vector, until a bit matching
 *	Value is found
 * 
 * Inputs:
 *	pBitVector		- pointer to bit vector to iterate through
 *	Value			- bit value to find
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	Index value where the first bit with the given value was found.
 *	If the callback given value was never found,
 *	then the return value = the size of the bit vector.
 * 
 ****************************************************************************/
uint32 
BitVectorFindFromEnd(
	IN	const BIT_VECTOR* const	pBitVector, 
	IN	uint8				Value );


#ifdef __cplusplus
}	/* extern "C" */
#endif


#endif	/* _IBA_PUBLIC_IBITVECTOR_H_ */
