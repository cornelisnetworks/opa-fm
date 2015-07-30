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


#ifndef _IBA_PUBLIC_IRESMAP_H_
#define _IBA_PUBLIC_IRESMAP_H_

#include "iba/public/datatypes.h"
#include "iba/public/ibitvector.h"


#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 *
 * Resource Map definition
 *
 * A Resourcse Map tracks the available resources in a BitVector and
 * assists in allocating, freeing and manipulating the resources
 * allocation attempts to use an expanding blocks approach
 * The BitVector is not grown until all resources in its scope are
 * allocated.  When allocation is performed it moves increasing through
 * the BitVector until the end is reached, then it re-uses entries from
 * the start.  This will tend to recycle the resources and avoid re-use
 * of recently used resources (which can be good for security but bad
 * for cache hit rates).
 * 
 * Resource numbers start at 0.
 */

typedef BITVEC_APPLY_FUNC RESMAP_APPLY_FUNC;


/* The resource map structure is allocated by the user and is opaque to them */
typedef struct _RESOURCE_MAP
{ 
	/* all fields below are private, use accessor functions to */
	/* manipulate this structure */
	BIT_VECTOR		m_ResList;
	uint32			m_FirstFree;	/* lowest which may be free */
	uint32			m_NextFree;		/* where to start search */
	uint32			m_NumReserved;	/* number of reserved entries */
	uint32			m_NumAllocated;	/* number of allocated entries */
} RESOURCE_MAP;


/****************************************************************************
 * resource map access methods
 * These methods are an attempt to follow the functionality of the C++ STL.
 * The names and behavior are obviously not exact matches, but hopefully,
 * close enough to make those familiar with the ANSI standard comfortable.
 ****************************************************************************/


/****************************************************************************
 * ResMapInitState
 * 
 * Description:
 *	This function performs basic resource map initialization.
 *	This function cannot fail.
 * 
 * Inputs:
 *	pResMap - Pointer to resource map
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void 
ResMapInitState(
	IN	RESOURCE_MAP* const	pResMap );


/****************************************************************************
 * ResMapInit
 * 
 * Description:
 *	This function initializes the resource map.
 *	all entries are initialized as free.
 * 
 * Inputs:
 *	pResMap		- Pointer to resource map
 *	InitialSize	- prefered initial number of resources
 *	GrowSize	- number of resources to increasing scope
 *					allocate when incrementally growing	the resource map
 *					(will round up to multiple of 32 or 64)
 *	MaxSize		- maximum number of resources to track, 0 -> no limit
 *	IsPageable	- TRUE indicates the resource map should use pageable memory
 *				FALSE indicates the resource map should used non-paged memory
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY - resource map is initialized to zero size
 * 
 ****************************************************************************/
FSTATUS
ResMapInit(
	IN	RESOURCE_MAP* const	pResMap,
	IN	const uint32	InitialSize,
	IN	const uint32	GrowSize,
	IN	const uint32	MaxSize,
	IN	boolean			IsPageable );


/****************************************************************************
 * ResMapDestroy
 * 
 * Description:
 *	This function deallocates all allocated memory associated with the resource map.
 *	The resource map is left initialized to zero size.
 *	This function does not fail.
 * 
 * Inputs:
 *	pResMap - Pointer to resource map
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void
ResMapDestroy(
	IN	RESOURCE_MAP* const	pResMap );


/****************************************************************************
 * ResMapGetSize
 *
 * Description:
 *	This function returns the current size of the resource map. 
 *	This function cannot fail.
 *	This indicates the range of resources being tracked presently
 *
 * Inputs:
 *	pResMap	- Pointer to resource map
 *
 * Outputs:
 *	None.
 *
 * Returns:
 *	present Size, in resources, of the resource map.
 *
 ****************************************************************************/
static __inline uint32
ResMapGetSize(
	IN	const RESOURCE_MAP* const	pResMap )
{
	DEBUG_ASSERT( pResMap );
	return BitVectorGetSize(&pResMap->m_ResList);
}

/****************************************************************************
 * ResMapGetNumReserved
 *
 * Description:
 *	This function returns the number of reserved entries in the resource map. 
 *	This function cannot fail.
 *
 * Inputs:
 *	pResMap	- Pointer to resource map
 *
 * Outputs:
 *	None.
 *
 * Returns:
 *	present number of reserved resources in the resource map.
 *
 ****************************************************************************/
static __inline uint32
ResMapGetNumReserved(
	IN	const RESOURCE_MAP* const	pResMap )
{
	DEBUG_ASSERT( pResMap );
	return pResMap->m_NumReserved;
}


/****************************************************************************
 * ResMapGetNumAllocated
 *
 * Description:
 *	This function returns the number of allocated entries in the resource map. 
 *	This function cannot fail.
 *	returned count does not include reserved resources.
 *
 * Inputs:
 *	pResMap	- Pointer to resource map
 *
 * Outputs:
 *	None.
 *
 * Returns:
 *	present number of allocated resources in the resource map.
 *
 ****************************************************************************/
static __inline uint32
ResMapGetNumAllocated(
	IN	const RESOURCE_MAP* const	pResMap )
{
	DEBUG_ASSERT( pResMap );
	return pResMap->m_NumAllocated;
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
	IN	const RESOURCE_MAP* const	pResMap );

/****************************************************************************
 * ResMapGetIsAllocated
 * 
 * Description:
 *	This function returns the status of the specified resource
 *	Range checking is NOT performed.  Provides constant time access.
 *	Internal function, not for general use.
 * 
 * Inputs:
 *	pResMap	- Pointer to resource map.
 *	Number	- Number of resource to check
 * 
 * Outputs:
 *	FALSE->free, TRUE-> allocated
 * 
 * Returns:
 *	None.
 * 
 * Notes:
 * 	prefered interface is ResMapIsAllocated
 * 
 ****************************************************************************/
static __inline boolean
_ResMapGetIsAllocated(
	IN	const RESOURCE_MAP* const	pResMap, 
	IN	const uint32		Number )
{
	DEBUG_ASSERT(pResMap);
	return ( BitVectorGet(&pResMap->m_ResList, Number) != 0);
}


/****************************************************************************
 * ResMapIsAllocated
 * 
 * Description:
 *	This function returns the status of the specified resource
 *	Range checking is performed.  Provides constant time access.
 * 
 * Inputs:
 *	pResMap		- Pointer to resource map.
 *	Number	- Number of resource to check
 * 
 * Outputs:
 * 
 * Returns:
 *	TRUE - allocated, FALSE - free or Number out of range
 * 
 ****************************************************************************/
boolean
ResMapIsAllocated(
	IN	const RESOURCE_MAP* const	pResMap, 
	IN	const uint32		Number );



/****************************************************************************
 * ResMapReserve
 * 
 * Description:
 *  Reserves a specific resource number
 * 
 * Inputs:
 *	pResMap		- pointer to resource map to iterate through
 *	Number		- resource number to reserve
 * 
 * Outputs:
 * 
 * Returns:
 *	FSUCCESS - Number now marked as allocated
 *	FINSUFFICIENT_RESOURCES  - Number already allocated
 *	FINSUFFICIENT_MEMORY  - unable to grow map
 * 
 ****************************************************************************/
FSTATUS 
ResMapReserve(
	IN	RESOURCE_MAP* const	pResMap,
	IN	uint32				Number );



/****************************************************************************
 * ResMapAllocate
 * 
 * Description:
 *  Locates a free resource Number
 * 
 * Inputs:
 *	pResMap		- pointer to resource map to iterate through
 *	pNumber		- where to store allocated resource number
 * 
 * Outputs:
 *	*pNumber	- resource allocated
 * 
 * Returns:
 *	FSUCCESS - *pNumber is Resource number (now marked as allocated)
 *	FINSUFFICIENT_RESOURCES  - no free resources
 *	FINSUFFICIENT_MEMORY  - unable to grow map
 * 
 ****************************************************************************/
FSTATUS 
ResMapAllocate(
	IN	RESOURCE_MAP* const	pResMap,
	OUT uint32						*pNumber );



/****************************************************************************
 * ResMapFree
 * 
 * Description:
 *	This function sets the given resource number to be free.
 *	should not be used to free reserved entries
 * 
 * Inputs:
 *	pResMap		- Pointer to resource map.
 *	Number		- Number of resource to free
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	FSUCCESS
 *	FINVALID_STATE - resource was already free
 * 
 ****************************************************************************/
FSTATUS 
ResMapFree(
	IN	RESOURCE_MAP* const	pResMap, 
	IN	const uint32	Number);



/****************************************************************************
 * ResMapApplyFuncAllocated
 * 
 * Description:
 *	This function calls the user supplied function for each allocated resource
 *	in the resource map, starting from the beginning of the array.
 * 
 *	The called function has the form:
 *		void ApplyFunc( uint32 Number, uint8 IsAllocated, void *Context );
 *	where:
 *		Number = resourcse number
 *		IsAllocated = 1/TRUE->allocated, 0/FALSE->free
 *		Context = user supplied context
 * 
 * Inputs:
 *	pResMap		- pointer to resource map to iterate through
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
ResMapApplyFuncAllocated(
	IN	const RESOURCE_MAP* const	pResMap, 
	IN	RESMAP_APPLY_FUNC	pfnCallback,
	IN	void* const			Context);

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif	/* _IBA_PUBLIC_IRESMAP_H_ */
