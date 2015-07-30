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


#include "iresmap.h"

///////////////////////////////////////////////////////////////////////////////
// resource map access methods
// These methods are an attempt to follow the functionality of the C++ STL.
// The names and behavior are obviously not exact matches, but hopefully,
// close enough to make those familiar with the ANSI standard comfortable.
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// ResMapInitState
// 
// Description:
//	This function performs basic resource map initialization.  This function cannot 
//	fail.
// 
// Inputs:
//	pResMap - Pointer to resource map
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void 
ResMapInitState(
	IN	RESOURCE_MAP* const	pResMap )
{
	ASSERT( pResMap );

	BitVectorInitState(&pResMap->m_ResList);
	pResMap->m_FirstFree = 0;
	pResMap->m_NextFree = 0;
	pResMap->m_NumReserved = 0;
	pResMap->m_NumAllocated = 0;
}



///////////////////////////////////////////////////////////////////////////////
// ResMapInit
// 
// Description:
//	This function initializes the resource map.  New bits in the resource map are
//	initialized to 0.
// 
// Inputs:
//	pResMap		- Pointer to resource map
//	InitialSize	- prefered initial number of bits
//	GrowSize	- number of bits to allocate when incrementally growing
//				the resource map (will round up to multiple of processor
//				word size)
//	MaxSize		- maximum number of bits in resource map, 0 -> no limit
//	IsPageable	- TRUE indicates the resource map should use pageable memory
//				FALSE indicates the resource map should used non-paged memory
// 
// Outputs:
//	None.
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY - resource map is initialized to zero size
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS
ResMapInit(
	IN	RESOURCE_MAP* const	pResMap,
	IN	const uint32	InitialSize,
	IN	const uint32	GrowSize,
	IN	const uint32	MaxSize,
	IN	boolean			IsPageable )
{
	ASSERT( pResMap );

	ResMapInitState( pResMap );
	return (BitVectorInit(&pResMap->m_ResList, InitialSize, GrowSize, MaxSize,
								IsPageable));
}


///////////////////////////////////////////////////////////////////////////////
// ResMapDestroy
// 
// Description:
//	This function deallocates all allocated memory associated with the resource map.
//	The resource map is left initialized to zero size.  This function does not fail.
// 
// Inputs:
//	pResMap - Pointer to resource map
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void
ResMapDestroy(
	IN	RESOURCE_MAP* const	pResMap )
{
	ASSERT( pResMap );

	BitVectorDestroy(&pResMap->m_ResList);
	pResMap->m_FirstFree = 0;
	pResMap->m_NextFree = 0;
	pResMap->m_NumReserved = 0;
	pResMap->m_NumAllocated = 0;

	return;
}


/****************************************************************************
 * ResMapAvailable
 *
 * Description:
 *	This function indicates if the resource map has any available resources
 *
 * Inputs:
 *	pResMap	- Pointer to resource map
 *
 * Outputs:
 *	None.
 *
 * Returns:
 *	TRUE - resources are available
 *	FALSE - no resources are available
 *
 ****************************************************************************/
boolean
ResMapAvailable(
	IN	const RESOURCE_MAP* const	pResMap )
{
	uint32 max_size;

	DEBUG_ASSERT( pResMap );
	max_size = BitVectorGetMaxSize(&pResMap->m_ResList);
	/* max_size ==0 -> no limit */
	return (! max_size
			|| pResMap->m_NumReserved + pResMap->m_NumAllocated < max_size);
}

///////////////////////////////////////////////////////////////////////////////
// ResMapIsAllocated
// 
// Description:
//	This function returns the status of the specified resource
//	Range checking is performed.  Provides constant time access.
// 
// Inputs:
//	pResMap		- Pointer to resource map.
//	Number	- Number of resource to check
// 
// Outputs:
// 
// Returns:
//	TRUE - allocated, FALSE - free or Number out of range
// 
///////////////////////////////////////////////////////////////////////////////
boolean
ResMapIsAllocated(
	IN	const RESOURCE_MAP* const	pResMap, 
	IN	const uint32		Number )
{
	ASSERT( pResMap );	// Other asserts are in ResMapGet

	// Range check
	if( Number >= BitVectorGetSize(&pResMap->m_ResList))
	{
		return FALSE;	// beyond what we have allocated/tracked
	}

	return _ResMapGetIsAllocated( pResMap, Number );
}


///////////////////////////////////////////////////////////////////////////////
// ResMapReserve
// 
// Description:
//  Reserves a specific resource number
// 
// Inputs:
//	pResMap		- pointer to resource map to iterate through
//	Number		- resource number to reserve
// 
// Outputs:
// 
// Returns:
//	FSUCCESS - Number now marked as allocated
//	FINSUFFICIENT_RESOURCES  - Number already allocated
//	FINSUFFICIENT_MEMORY  - unable to grow map
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
ResMapReserve(
	IN	RESOURCE_MAP* const	pResMap,
	IN	uint32				Number )
{
	FSTATUS Status;

	// check Number is not already allocated or reserved
	if (BitVectorGetSize(&pResMap->m_ResList) > Number)
	{
		if (BitVectorGet(&pResMap->m_ResList, Number))
		{
			return FINSUFFICIENT_RESOURCES;
		}
	}
	Status = BitVectorSet(&pResMap->m_ResList, Number, 1);
	if (Status == FSUCCESS)
	{
		// Leave FirstFree and NextFree alone, won't hurt much
		++(pResMap->m_NumReserved);
	}
	return Status;
}


///////////////////////////////////////////////////////////////////////////////
// ResMapAllocate
// 
// Description:
//  Locates a free resource Number
// 
// Inputs:
//	pResMap		- pointer to resource map to iterate through
//	pNumber		- where to store allocated resource number
// 
// Outputs:
//	*pNumber	- resource allocated
// 
// Returns:
//	FSUCCESS - *pNumber is Resource number (now marked as allocated)
//	FINSUFFICIENT_RESOURCES  - no free resources
//	FINSUFFICIENT_MEMORY  - unable to grow map
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
ResMapAllocate(
	IN	RESOURCE_MAP* const	pResMap,
	OUT uint32				*pNumber )
{
	uint32 Size;
	uint32 Number;
	FSTATUS Status;

	ASSERT(pResMap);

	// no use doing all this searching if no resources available
	if (! ResMapAvailable(pResMap))
		return FINSUFFICIENT_RESOURCES;

	Size = BitVectorGetSize(&pResMap->m_ResList);
	// search from m_NextFree to end of present size
	Number = BitVectorFindFromIndex(&pResMap->m_ResList, FALSE, pResMap->m_NextFree);
	if (Number < Size)
	{
		Status = BitVectorSet(&pResMap->m_ResList, Number, 1);
		if (Status != FSUCCESS)
		{
			return Status;
		}
		goto found;
	}
	// search from start of list
	if (pResMap->m_FirstFree < pResMap->m_NextFree)
	{
		// search from m_FirstFree to m_NextFree
		Number = BitVectorFindRange(&pResMap->m_ResList, FALSE, pResMap->m_FirstFree, pResMap->m_NextFree);
		if (Number == pResMap->m_NextFree) Number = Size;
	} else {
		// expand to grow list
		Number = Size;
	}
	Status = BitVectorSet(&pResMap->m_ResList, Number, 1);
	if (Status != FSUCCESS)
	{
		return Status;
	}
	pResMap->m_FirstFree = Number+1;
found:
	pResMap->m_NextFree = Number+1;
	++(pResMap->m_NumAllocated);
	*pNumber = Number;
	return FSUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// ResMapFree
// 
// Description:
//	This function sets the given resource number to be free.
//	should not be used to free reserved entries
// 
// Inputs:
//	pResMap		- Pointer to resource map.
//	Number		- Number of resource to free
// 
// Outputs:
//	None.
// 
// Returns:
//	FSUCCESS
//	FINVALID_STATE - resource was already free
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
ResMapFree(
	IN	RESOURCE_MAP* const	pResMap, 
	IN	const uint32	Number)
{
	FSTATUS Status;

	if (Number >= BitVectorGetSize(&pResMap->m_ResList)
		|| BitVectorGet(&pResMap->m_ResList, Number) == 0)
	{
		return FINVALID_STATE;
	}
	DEBUG_ASSERT(pResMap->m_NumAllocated);

	Status = BitVectorSet(&pResMap->m_ResList, Number, 0);
	ASSERT(Status == FSUCCESS);
	if (Number < pResMap->m_FirstFree)
		pResMap->m_FirstFree = Number;
	--(pResMap->m_NumAllocated);
	return Status;
}


///////////////////////////////////////////////////////////////////////////////
// ResMapApplyFuncAllocated
// 
// Description:
//	This function calls the user supplied function for each allocated resource
//	in the resource map, starting from the beginning of the array.
// 
//	The called function has the form:
//		void ApplyFunc( uint32 Number, uint8 IsAllocated, void *Context );
//	where:
//		Number = resourcse number
//		IsAllocated = 1/TRUE->allocated, 0/FALSE->free
//		Context = user supplied context
// 
// Inputs:
//	pResMap		- pointer to resource map to iterate through
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
ResMapApplyFuncAllocated(
	IN	const RESOURCE_MAP* const	pResMap, 
	IN	RESMAP_APPLY_FUNC	pfnCallback,
	IN	void* const			Context)
{
	BitVectorApplyFunc(&pResMap->m_ResList, pfnCallback, Context);
}
