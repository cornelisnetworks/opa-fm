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

/****************************************************************************
 *
 * Map
 *
 ****************************************************************************/

#ifndef _IBA_PUBLIC_IMAP_H_INCLUDED_
#define _IBA_PUBLIC_IMAP_H_INCLUDED_

#include "iba/public/datatypes.h"
#include "iba/public/imath.h"
#include "iba/public/igrowpool.h"

#ifdef __cplusplus
extern "C" {
#endif


/* enable this #define for more debug information */
#ifdef IB_DEBUG
#define MAP_DEBUG 1
#else
#define MAP_DEBUG 0
#endif

#define MAP_RECURSION_THRESHOLD 0
#define MAP_CONSOLE_DISPLAY_DEPTH_MAX 7
#define MAP_START_SIZE 256
#define MAP_GROW_SIZE 256
#define MAP_MAX_MIN_DELTA 2

/* define the various rotations */
typedef enum _MAP_ROTDIR
{
	MAP_ROTNONE,		/* no rotation */
	MAP_ROTL,			/* rotate left */
	MAP_ROTR,			/* rotate right */
	MAP_ROTDL,			/* double rotate left */
	MAP_ROTDR,			/* double rotate right */
	MAP_RECURSIVE		/* too complex for simple rotations */

} MAP_ROTDIR;

/* define the map linkage structure */
typedef struct _MAP_ITEM
{
	struct _MAP_ITEM	*m_pLeft;		/* DO NOT USE!! */
	struct _MAP_ITEM	*m_pRight;		/* DO NOT USE!! */
	struct _MAP_ITEM	*m_pUp;			/* DO NOT USE!! */
	int8				m_DLMin;		/* DO NOT USE!! */
	int8				m_DLMax;		/* DO NOT USE!! */
	int8				m_DRMin;		/* DO NOT USE!! */
	int8				m_DRMax;		/* DO NOT USE!! */
	uint64				m_Key;			/* DO NOT USE!! */
	void				*m_pObj;		/* DO NOT USE!! */

} MAP_ITEM, *PMAP_ITEM;

/* define the map structure for global map resources */
typedef struct _MAP_RES
{
	MAP_ITEM			m_End;			/* DO NOT USE!! */
	uint32				m_Count;		/* DO NOT USE!! */
	boolean				m_DelToggle;	/* DO NOT USE!! */
	GROW_POOL			m_Pool;			/* DO NOT USE!! */

} MAP_RES, *PMAP_RES;

/* Define the iterator used to traverse the tree */
typedef struct _MAP_ITERATOR
{
	MAP_ITEM	*pItem;
} MAP_ITERATOR;


/* internal function prototypes not for use by user's of map */
MAP_ITEM* MapItorNext( IN MAP_ITERATOR* const pItor );
MAP_ITEM* MapItorPrev( IN MAP_ITERATOR* const pItor );
void MapUpdateDepthsNoCheck( IN MAP_ITEM* const pItem );
MAP_ITEM* MapItorInit(	IN MAP_ITEM* const pItem,
						IN MAP_ITERATOR * const pItor );
MAP_ITEM** MapItemGetPtrAbove( IN MAP_ITEM* const pItem );

/****************************************************************************
 *
 * MapItorItem
 *
 * Description:
 *  Returns a pointer to the MAP_ITEM pointed to by the iterator.
 *  This function should not be called by user's of map.
 *
 * Inputs:
 *  pItor - pointer to a map iterator
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  Pointer to the MAP_ITEM to which the iterator points
 *
 ****************************************************************************/
static __inline MAP_ITEM*
MapItorItem(
	IN const MAP_ITERATOR* const pItor )
{
	return( pItor->pItem );
}

/****************************************************************************
 *
 * MapDebugGetRootKey
 *
 * Description:
 *  Returns the key value of the current root of the map.
 *  This function is for debugging only and should not be called by
 *  general users of MAP_RES.
 *
 *
 * Inputs:
 *  pMap	- pointer to a MAP_RES
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  TRUE if the item is a leaf
 *  FALSE otherwise
 *
 ****************************************************************************/
static __inline uint64
MapDebugGetRootKey(
	IN const MAP_RES* const pMap )
{
	ASSERT( pMap );
	ASSERT( pMap->m_End.m_pLeft );
	return( pMap->m_End.m_pLeft->m_Key );
}

/****************************************************************************
 *
 * MapRoot
 *
 * Description:
 *  Returns the current root of the map.
 *  This function should not be called by user's of map.
 *
 * Inputs:
 *  pMap	- pointer to a MAP_RES
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  TRUE if the item is a leaf
 *  FALSE otherwise
 *
 ****************************************************************************/
static __inline MAP_ITEM*
MapRoot(
	IN const MAP_RES* const pMap )
{
	ASSERT( pMap );
	return( pMap->m_End.m_pLeft );
}

/****************************************************************************
 *
 * MapItorGetPtrAbove
 *
 * Description:
 *  Returns a pointer to a the pointer that points to the node at
 *  the Iterator.
 *
 * Inputs:
 *  pItor - pointer to a map iterator
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  pointer to a pointer to the node at the iterator
 *
 ****************************************************************************/
static __inline MAP_ITEM**
MapItorGetPtrAbove(
	IN const MAP_ITERATOR* const pItor )
{
	return( MapItemGetPtrAbove( pItor->pItem ) );
}

/****************************************************************************
 *
 * MapItorUp
 *
 * Description:
 *  Moves the iterator one step toward the root.
 *  Returns a pointer to the new MAP_ITEM pointed to by the iterator.
 *  This function should not be called by user's of map.
 *
 * Inputs:
 *  pItor - pointer to a map iterator
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  Pointer to the MAP_ITEM to which the iterator points
 *
 ****************************************************************************/
static __inline MAP_ITERATOR*
MapItorUp(
	IN MAP_ITERATOR* const pItor )
{
	pItor->pItem = pItor->pItem->m_pUp;
	return( pItor );
}

/****************************************************************************
 *
 * MapSetRoot
 *
 * Description:
 *  Sets the root of the map.
 *  This function should not be called by user's of map.
 *
 * Inputs:
 *  pMap	- pointer to a MAP_RES
 *  pItem	- pointer to the MAP_RES item.  This pointer may be NULL.
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  TRUE if the item is a leaf
 *  FALSE otherwise
 *
 ****************************************************************************/
static __inline void
MapSetRoot(
	IN MAP_RES* const pMap,
	IN MAP_ITEM* const pItem )
{
	ASSERT( pMap );
	pMap->m_End.m_pLeft = pItem;
	if( pItem )
		pItem->m_pUp = &pMap->m_End;
}

/****************************************************************************
 *
 * MapCount
 *
 * Description:
 *  Returns the number of items in the map.
 *
 * Inputs:
 *  pMap	- pointer to a MAP_RES
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  TRUE if the item is a leaf
 *  FALSE otherwise
 *
 ****************************************************************************/
static __inline uint32
MapCount(
	IN const MAP_RES* const pMap )
{
	ASSERT( pMap );
	return( pMap->m_Count );
}

/****************************************************************************
 *
 * MapObj
 *
 * Description:
 *  Returns the user object pointed to by an iterator.
 *
 * Inputs:
 *  pItor	- MAP_ITERATOR
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  pointer to the user object
 *
 ****************************************************************************/
static __inline void*
MapObj(
	IN const MAP_ITERATOR* const pItor )
{
	ASSERT( pItor );
	return( pItor->pItem->m_pObj );
}

/****************************************************************************
 *
 * MapKey
 *
 * Description:
 *  Returns the key pointed to by an iterator.
 *
 * Inputs:
 *  Itor			- MAP_ITERATOR
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  key value of the object pointed to by the iterator
 *
 ****************************************************************************/
static __inline uint64
MapKey(
	IN const MAP_ITERATOR* const pItor )
{
	ASSERT( pItor );
	return( pItor->pItem->m_Key );
}


/****************************************************************************
 *
 * MapIsLeaf
 *
 * Description:
 *  Returns true if the specified MAP_ITEM is a leaf.
 *  This function should not be called by user's of map.
 *
 * Inputs:
 *  pItem	- pointer to a MAP_ITEM
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  TRUE if the item is a leaf
 *  FALSE otherwise
 *
 ****************************************************************************/
static __inline boolean
MapIsLeaf(
	IN const MAP_ITEM* const pItem )
{
	ASSERT( pItem );
	return( (boolean)((pItem->m_pLeft == NULL) && (pItem->m_pRight == NULL)) );
}

/****************************************************************************
 *
 * MapIsRoot
 *
 * Description:
 *  Returns true if the specified MAP_ITEM is the root of the given MAP_RES
 *  This function should not be called by user's of map.
 *
 * Inputs:
 *  pMap	- pointer ot a MAP_ITEM
 *  pItem	- pointer to a MAP_ITEM
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  TRUE if the item is the root
 *  FALSE otherwise
 *
 ****************************************************************************/
static __inline boolean
MapIsRoot(
	IN const MAP_RES* const pMap,
	IN const MAP_ITEM* const pItem)
{
	ASSERT( pMap );
	return( (boolean)(pMap->m_End.m_pLeft == pItem) );
}

/****************************************************************************
 *
 * MapItorIsLeaf
 *
 * Description:
 *  Returns true if the item pointed to by the iterator is a leaf.
 *  This function should not be called by user's of map.
 *
 * Inputs:
 *  pItor	- pointer to a MAP_ITERATOR
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  TRUE if the iterator's item is a leaf
 *  FALSE otherwise
 *
 ****************************************************************************/
static __inline boolean
MapItorIsLeaf(
	IN const MAP_ITERATOR* const pItor )
{
	ASSERT( pItor );
	return( (boolean)((pItor->pItem->m_pLeft == NULL) &&
		(pItor->pItem->m_pRight == NULL) ));
}

/****************************************************************************
 *
 * MapItorDeleteLeaf
 *
 * Description:
 *  Performs a simple delete of the item pointed to by the given iterator.
 *  The iterator must point to a leaf item.
 *  The iterator may point to the root node of this iterator.
 *  The balances in the tree nodes are not adjusted!
 *  This internal function should not be called by users of MAP_RES.
 *
 * Inputs:
 *  pItor		- pointer to a MAP_ITERATOR object
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  None
 *
 ****************************************************************************/
static __inline void
MapItorDeleteLeaf(
	IN MAP_ITERATOR* const pItor )
{
	/* The pointer that points to the leaf item must be set to NULL */
	/* or the node above the node to be deleted, determine when the */
	/* node to delete hangs to the left or right. */
	ASSERT( pItor );
	ASSERT( MapIsLeaf( pItor->pItem ) );
	*(MapItorGetPtrAbove( pItor )) = NULL;
	/* put the iterator back into the map now that the node */
	/* it points to has been deleted. */
	MapItorUp( pItor );
}

/****************************************************************************
 *
 * MapItorDeleteRightHalfLeaf
 *
 * Description:
 *  Performs a simple delete of the item pointed to by the given iterator.
 *  The iterator must point to an item that has a NULL child on the right.
 *  The iterator can NOT point to the root node of this iterator.
 *  The iterator may point to a true leaf.
 *  The balances in the tree nodes is not adjusted!
 *  This internal function should not be called by users of MAP_RES.
 *
 * Inputs:
 *  pItor		- pointer to a MAP_ITERATOR object
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  None
 *
 ****************************************************************************/
static __inline void
MapItorDeleteRightHalfLeaf(
	IN MAP_ITERATOR* const pItor )
{
	ASSERT( pItor );
	ASSERT( pItor->pItem->m_pRight == NULL );
	*(MapItorGetPtrAbove( pItor )) = pItor->pItem->m_pLeft;
	if( pItor->pItem->m_pLeft != NULL )
		pItor->pItem->m_pLeft->m_pUp = pItor->pItem->m_pUp;
	/* put the iterator back into the map now that the node */
	/* it points to has been deleted. */
	MapItorUp( pItor );
}

/****************************************************************************
 *
 * MapItorDeleteLeftHalfLeaf
 *
 * Description:
 *  Performs a simple delete of the item pointed to by the given iterator.
 *  The iterator must point to an item that has a NULL child on the left.
 *  The iterator can NOT point to the root node of this iterator.
 *  The iterator may point to a true leaf.
 *  The balances in the tree nodes is not adjusted!
 *  This internal function should not be called by users of MAP_RES.
 *
 * Inputs:
 *  pItor		- pointer to a MAP_ITERATOR object
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  None
 *
 ****************************************************************************/
static __inline void
MapItorDeleteLeftHalfLeaf(
	IN MAP_ITERATOR * const pItor )
{
	ASSERT( pItor );
	ASSERT( pItor->pItem->m_pLeft == NULL );
	*(MapItorGetPtrAbove( pItor )) = pItor->pItem->m_pRight;
	if( pItor->pItem->m_pRight != NULL )
		pItor->pItem->m_pRight->m_pUp = pItor->pItem->m_pUp;
	/* put the iterator back into the map now that the node */
	/* it points to has been deleted. */
	MapItorUp( pItor );
}


/****************************************************************************
 *
 * MapItorLeft
 *
 * Description:
 *  Step the iterator one node down to the left.
 *  This function should not be called by user's of map.
 *
 * Inputs:
 *  pItor - pointer to a map iterator
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  Pointer to the MAP_ITEM to which the iterator moved
 *
 ****************************************************************************/
static __inline MAP_ITEM*
MapItorLeft(
	IN MAP_ITERATOR* const pItor )
{
	pItor->pItem = pItor->pItem->m_pLeft;
	return( pItor->pItem );
}

/****************************************************************************
 *
 * MapItemInit
 *
 * Description:
 *  Initializes a MAP_ITEM object.
 *  This function must be called before using a new MAP_ITEM.
 *  This internal function should not be called by users of MAP_RES.
 *
 * Inputs:
 *  pItem	- pointer to a MAP_ITEM.
 *  Key		- key value associated with this MAP_ITEM.
 *  pObj	- user object associated with theis MAP_ITEM.
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  None
 *
 ****************************************************************************/
static __inline void
MapItemResetBalances(
	IN MAP_ITEM* const pItem )
{
	pItem->m_DLMin = -1;
	pItem->m_DLMax = -1;
	pItem->m_DRMin = -1;
	pItem->m_DRMax = -1;
}

/****************************************************************************
 *
 * MapItorRight
 *
 * Description:
 *  Step the iterator one node down to the right.
 *  This function should not be called by user's of map.
 *
 * Inputs:
 *  pItor - pointer to a map iterator
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  Pointer to the MAP_ITEM to which the iterator moved
 *
 ****************************************************************************/
static __inline MAP_ITEM*
MapItorRight(
	IN MAP_ITERATOR* const pItor )
{
	pItor->pItem = pItor->pItem->m_pRight;
	return( pItor->pItem );
}

/****************************************************************************
 *
 * MapItorAtRoot
 *
 * Description:
 *  Returns true if this iterator points to the root node
 *
 * Inputs:
 *  pItor - pointer to a map iterator
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  TRUE - The iterator points to the root node
 *  FALSE otherwise
 *
 ****************************************************************************/
static __inline boolean
MapItorAtRoot(
	IN const MAP_ITERATOR* const pItor )
{
	return( (boolean)(pItor->pItem->m_pUp == pItor->pItem) );
}

/****************************************************************************
 *
 * MapIsItorValid
 *
 * Description:
 *  Returns TRUE if the MAP_ITERATOR points to an object in the map.
 *  Returns FALSE if the iterator points outside the map or to the logical
 *  end of the map (which is of course outstide the map too)
 *
 * Inputs:
 *  Itor			- MAP_ITERATOR
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  key value of the object pointed to by the iterator
 *
 ****************************************************************************/
static __inline boolean
MapIsItorValid(
	IN const MAP_ITERATOR* const pItor )
{
	if( pItor->pItem != NULL )
		return( !MapItorAtRoot( pItor ) );
	else
		return( FALSE );
}

/****************************************************************************
 *
 * MapRotRight
 *
 * Description:
 *  Rotate the subtree pointed to by *ppRoot to the right.  This transform
 *  can rebalance a -2 LL
 *
 * Inputs:
 *  ppRoot - pointer to a pointer to a MAP_ITEM
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  None
 *
 ****************************************************************************/
static __inline void
MapRotRight(
	IN MAP_ITEM *pItem )
{
	MAP_ITEM **ppRoot = MapItemGetPtrAbove( pItem );

    ASSERT( pItem );
    ASSERT( ppRoot );
	ASSERT( *ppRoot );

	/* point to the new root */
	/* which is the left leaf of the old root */
	*ppRoot = pItem->m_pLeft;
	pItem->m_pLeft->m_pUp = pItem->m_pUp;
	
	/* Set old root's left tree to the new root's old right tree */
	pItem->m_pLeft = (*ppRoot)->m_pRight;
	if( pItem->m_pLeft )
		pItem->m_pLeft->m_pUp = pItem;

	/* the right subtree of the new root becomes the old root */
	/* the unblanaced node */
	(*ppRoot)->m_pRight = pItem;
	if( (*ppRoot)->m_pRight )
		(*ppRoot)->m_pRight->m_pUp = *ppRoot;

	MapUpdateDepthsNoCheck( pItem );
	MapUpdateDepthsNoCheck( *ppRoot );

}

/****************************************************************************
 *
 * MapRotLeft
 *
 * Description:
 *  Rotate the subtree pointed to by *ppRoot to the left.  This transform
 *  can rebalance a +2 RR
 *
 * Inputs:
 *  ppRoot - pointer to a pointer to a MAP_ITEM
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  None
 *
 ****************************************************************************/
static __inline void
MapRotLeft(
	IN MAP_ITEM *pItem )
{
	MAP_ITEM **ppRoot = MapItemGetPtrAbove( pItem );

    ASSERT( pItem );
	ASSERT( ppRoot );
	ASSERT( *ppRoot );

	/* point the new root */
	/* which is the right leaf of the old root */
	*ppRoot = pItem->m_pRight;
	pItem->m_pRight->m_pUp = pItem->m_pUp;
	
	/* setup the left subtree of the old root */
	/* to be the right subtree of the new root */
	pItem->m_pRight = (*ppRoot)->m_pLeft;
	if( pItem->m_pRight )
		pItem->m_pRight->m_pUp = pItem;

	/* the right subtree of the new root becomes the old root */
	/* the unblanaced node */
	(*ppRoot)->m_pLeft = pItem;
	if( (*ppRoot)->m_pLeft )
		(*ppRoot)->m_pLeft->m_pUp = *ppRoot;

	MapUpdateDepthsNoCheck( pItem );
	MapUpdateDepthsNoCheck( *ppRoot );
}

/****************************************************************************
 *
 * MapNext
 *
 * Description:
 *  Sets a map iterator to the logical next item in the map.
 *
 * Inputs:
 *  pItor - pointer to a map iterator
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  Pointer to the user object at the next map location
 *
 ****************************************************************************/
static __inline void*
MapNext(
	IN MAP_ITERATOR* const pItor )
{
	MAP_ITEM *pItem;
	ASSERT( pItor );
	pItem = MapItorNext( pItor );
	if( pItem != NULL )
		return( pItem->m_pObj );
	else
		return NULL;
}

/****************************************************************************
 *
 * MapPrev
 *
 * Description:
 *  Sets a map iterator to the previous item in the map.
 *
 * Inputs:
 *  pItor - pointer to a map iterator
 *
 * Outputs:
 *  None
 *
 * Returns:
 *  Pointer to the user object at the next map location
 *
 ****************************************************************************/
static __inline void*
MapPrev(
	IN MAP_ITERATOR* const pItor )
{
	MAP_ITEM *pItem;
	ASSERT( pItor );
	pItem = MapItorPrev( pItor );
	if( pItem != NULL )
		return( pItem->m_pObj );
	else
		return NULL;
}



void MapInitState(	IN MAP_RES * const pMap );

FSTATUS MapInit(	IN MAP_RES * const pMap );

void MapDestroy(	IN MAP_RES * const pMap );

FSTATUS MapInsert(	IN MAP_RES* const pMap,
					IN const uint64 Key,
					IN const void* const pObj );

void* MapHead(		IN const MAP_RES* const pMap,
					IN MAP_ITERATOR* const pItor );

void* MapRemoveHead(	IN MAP_RES* const pMap );

void* MapTail(		IN const MAP_RES* const pMap,
					IN MAP_ITERATOR* const pItor );

void* MapRemoveTail(	IN MAP_RES* const pMap );

FSTATUS MapRemove(	IN MAP_RES* const pMap,
					IN const uint64 Key );

void MapConsoleDump( IN const MAP_RES* const pMap );

void* MapGet(	IN const MAP_RES* const pMap,
				IN const uint64 Key );


/*FSTATUS MapDebugCheckDepth( IN const MAP_RES * const pMap ); */

#ifdef __cplusplus
}
#endif 

#endif /* _IBA_PUBLIC_IMAP_H_INCLUDED */
