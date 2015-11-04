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
//////////////////////////////////////////////////////////////////////////////
//
// Map
//
// Map is an associative array.  By providing a key, the caller can retrieve
// an object from the map.  All objects in the map have an associated key,
// as specified by the caller when the object was inserted into the map.
// In addition to random access, the caller can traverse the map much like
// a linked list, either forwards from the first object or backwards from
// the last object.  The objects in the map are always traversed in
// order since the nodes are stored sorted.
// 
// This implementation of Map uses a balanced binary tree.
//
// There are several states of interest in a balanced binary tree.
//
// There are four cases where a subtree is considered balanced.
// Above each diagram is the balance represented by an integer.
//
//      0          -1      +1                         0
//
//		B           B       A         Trivial case:   A
//     / \         /         |  
//    A   C       A           B
//
//
//
// There are four cases after an insertion where a subtree is considered
// unbalanced:
//
//         -2        -2        +2         +2
//         (LL)      (LR)      (RR)       (RL)
//
//          C         C         A          A
//         /         /           \          |  
//        B         A             B          C
//       /           \             \        /
//      A             B             C      B
//
//
// Because the tree is balanced as necessary after every insertion, the four
// cases above are the ONLY four cases of imbalance in the tree.
//
// Each of the above cases must be reblanced to look like:
//
//
//        B
//       / |  
//      A   C
//
//
//  
// 
//////////////////////////////////////////////////////////////////////////////

#include "string.h"
#include "imap.h"
#include "idebug.h"


//////////////////////////////////////////////////////////////////////////////
//
// Local function prototypes
// These functions should not be called by users of MAP_RES
//
//////////////////////////////////////////////////////////////////////////////
void MapDoubleRight( IN MAP_ITEM* const ppRoot );
void MapDoubleLeft( IN MAP_ITEM* const ppRoot );
MAP_ITEM* MapItorProbeLeft( IN MAP_ITERATOR* const pItor );
MAP_ITEM* MapItorProbeRight( IN MAP_ITERATOR* const pItor );
uint32 MapConsoleDumpDepth(	IN const MAP_ITEM* const pItem,
						IN const uint8 ShowDepth,
						IN const uint8 CurrentDepth,
						IN uint32 Total );
void MapConsoleDumpSubtree( IN const MAP_ITEM* const pItem );

void MapConsolePad(	IN const uint8 ShowDepth,
					IN const boolean Leading,
					IN const uint32 FudgeFactor,
					IN const boolean Dashless );
boolean MapItorShouldPropogateForDelete( IN MAP_ITERATOR * const pItor );
MAP_ROTDIR MapItemCheckRot( IN const MAP_ITEM * const pItem );
boolean MapUpdateDepths( IN MAP_ITEM* const pItem );
void MapItorWalkBackAndBalance(	IN MAP_ITERATOR* const pItor );
void MapItemExchange(
	IN MAP_ITEM* const pOldItem,
	IN MAP_ITEM* const pNewItem );
void MapItorRecursiveBalance(	IN MAP_ITERATOR * const pItor );
void MapItorDeleteLeaf( IN MAP_ITERATOR * const pItor );
void MapItorDeleteLeftHalfLeaf( IN MAP_ITERATOR * const pItor );
void MapItorDeleteRightHalfLeaf( IN MAP_ITERATOR * const pItor );
void MapItemInsertLeft(
	IN MAP_ITEM* const pInsertItem,
	IN MAP_ITEM* const pAtItem );
void MapItemInsertRight(
	IN MAP_ITEM* const pInsertItem,
	IN MAP_ITEM* const pAtItem );
void
MapItemInit(
	IN MAP_ITEM* const pItem,
	IN const uint64 Key,
	IN const void* const pObj );
void
MapItorWalkBackAndBalanceToItem(
	IN MAP_ITERATOR* const pItor,
	IN MAP_ITEM* const pItem );

//////////////////////////////////////////////////////////////////////////////
//
// MapInitState
//
// Description:
//  Initializes the state of a MAP_RES object.
//
// Inputs:
//  pMap	- pointer to a MAP_RES
//
// Outputs:
//  None
//
// Returns:
//  None
//
//////////////////////////////////////////////////////////////////////////////
void
MapInitState(
	IN MAP_RES* const pMap )
{
	ASSERT( pMap );

	GrowPoolInitState( &pMap->m_Pool );
	MapItemInit( &pMap->m_End, 0, NULL );
	// special setup for the root node
	pMap->m_End.m_pUp = &pMap->m_End;
	pMap->m_Count = 0;
}

//////////////////////////////////////////////////////////////////////////////
//
// MapInit
//
// Description:
//  Initializes a MAP_RES object.
//  This function must be called before using a new MAP_RES.
//
// Inputs:
//  pMap	- pointer to a MAP_RES
//
// Outputs:
//  None
//
// Returns:
//  None
//
//////////////////////////////////////////////////////////////////////////////
FSTATUS
MapInit(
	IN MAP_RES* const pMap )
{
	FSTATUS Status;

	MapInitState( pMap );
	pMap->m_DelToggle = FALSE;
	Status = GrowPoolInit( &pMap->m_Pool,
					MAP_START_SIZE,		// minimum number of objects to allocate
					sizeof( MAP_ITEM),	// size of object
					MAP_GROW_SIZE,		// minimum number of objects to grow
					NULL,	// no state initialization needed
					NULL,	// no initializer needed
					NULL,	// no destructor needed
					NULL );	// no context needed
	return Status;
}

//////////////////////////////////////////////////////////////////////////////
//
// MapDestroy
//
// Description:
//  Destroys a MAP_RES object.
//  This function must be called when finished with a MAP_RES.
//
// Inputs:
//  pMap	- pointer to a MAP_RES
//
// Outputs:
//  None
//
// Returns:
//  None
//
//////////////////////////////////////////////////////////////////////////////
void
MapDestroy(
	IN MAP_RES* const pMap )
{
	MAP_ITERATOR Itor;
	MAP_ITEM *pItem;

	// all objects must be returned to the resource pool
	// delete leaves until there is nothing left in the map

	if( MapRoot( pMap ) )
	{
		pItem = MapItorInit( MapRoot( pMap ), &Itor );

		while( !MapIsLeaf( pItem ) )
		{
			pItem = MapItorProbeLeft( &Itor );

			do
			{
				// Because we're deleting from right to left,
				// every node we encounter will either be a leaf
				// or a left half-leaf.
				// Obviously we don't bother with rebalancing.
				if( MapItorIsLeaf( &Itor ) )
				{
					MapItorDeleteLeaf( &Itor );

					// return this item to the pool
					GrowPoolPut( &pMap->m_Pool, pItem );
				}

			} while( (pItem = MapItorNext( &Itor )) != NULL );

			pItem = MapItorInit( MapRoot( pMap ), &Itor );
		}

		// now, put the root node itself in the resource pool
		// if it hasn't been deleted already
		if( MapRoot( pMap ) )
			GrowPoolPut( &pMap->m_Pool, MapRoot( pMap ) );
	}

	GrowPoolDestroy( &pMap->m_Pool );
	pMap->m_Count = 0;
	MapSetRoot( pMap, NULL );
}


//////////////////////////////////////////////////////////////////////////////
//
// MapItemInit
//
// Description:
//  Initializes a MAP_ITEM object.
//  This function must be called before using a new MAP_ITEM.
//  This internal function should not be called by users of MAP_RES.
//
// Inputs:
//  pItem	- pointer to a MAP_ITEM.
//  Key		- key value associated with this MAP_ITEM.
//  pObj	- user object associated with theis MAP_ITEM.
//
// Outputs:
//  None
//
// Returns:
//  None
//
//////////////////////////////////////////////////////////////////////////////
void
MapItemInit(
	IN MAP_ITEM * const pItem,
	IN const uint64 Key,
	IN const void* const pObj )
{
	pItem->m_Key = Key;
	pItem->m_pObj = (void *)pObj;
	pItem->m_pLeft = NULL;
	pItem->m_pRight = NULL;
	pItem->m_pUp = NULL;
	MapItemResetBalances( pItem );
}


//////////////////////////////////////////////////////////////////////////////
//
// MapItemGetPtrAbove
//
// Description:
//  Returns a pointer to a the pointer that points to the node at
//  the Iterator.
//
// Inputs:
//  pItor - pointer to a map iterator
//
// Outputs:
//  None
//
// Returns:
//  pointer to a pointer to the node at the iterator
//
//////////////////////////////////////////////////////////////////////////////
MAP_ITEM**
MapItemGetPtrAbove(
	IN MAP_ITEM* const pItem )
{
	MAP_ITEM *pAbove;
	ASSERT(pItem);
	pAbove = pItem->m_pUp;
	if( pAbove->m_pLeft == pItem )
		return( &pAbove->m_pLeft );
	else
	{
		ASSERT( pAbove->m_pRight == pItem );
		return( &pAbove->m_pRight );
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// MapItemCheckRot
//
// Description:
//  Returns the type of rotation, if any, necessary at the given item.
//  This internal function should not be called by users of MAP_RES.
//
// Inputs:
//  pItem - pointer to a MAP_ITEM
//
// Outputs:
//  None
//
// Returns:
//  MAP_ROTR - rotate right required
//  MAP_ROTDR - rotate double right required
//  MAP_ROTL - rotate left required
//  MAP_ROTDL - rotate double left required
//  MAP_ROTNONE - no rotation required
//  MAP_RECURSIVE - recursive rebalance required
//
//////////////////////////////////////////////////////////////////////////////
MAP_ROTDIR
MapItemCheckRot(
	IN const MAP_ITEM* const pItem )
{
#if MAP_DEBUG
	if( pItem->m_DLMax - pItem->m_DRMin > MAP_MAX_MIN_DELTA )
	{
		DbgOut("\n\aERROR - DLMax - DRMin > %u for key = 0x%"PRIx64,
			MAP_MAX_MIN_DELTA, pItem->m_Key );
		MapConsoleDumpSubtree( pItem );
		ASSERT( FALSE );
		return( MAP_ROTNONE );
	}
	if( pItem->m_DRMax - pItem->m_DLMin > MAP_MAX_MIN_DELTA )
	{
		DbgOut("\n\aERROR - DRMax - DLMin > %u for key = 0x%"PRIx64,
			MAP_MAX_MIN_DELTA, pItem->m_Key );
		MapConsoleDumpSubtree( pItem );
		ASSERT( FALSE );
		return( MAP_ROTNONE );
	}
	if( pItem->m_DLMax - pItem->m_DLMin > MAP_MAX_MIN_DELTA - 1 )
	{
		DbgOut("\n\aERROR - DLMax - DLMin > %u for key = 0x%"PRIx64,
			MAP_MAX_MIN_DELTA - 1, pItem->m_Key );
		MapConsoleDumpSubtree( pItem );
		ASSERT( FALSE );
		return( MAP_ROTNONE );
	}
	if( pItem->m_DRMax - pItem->m_DRMin > MAP_MAX_MIN_DELTA - 1 )
	{
		DbgOut("\n\aERROR - DRMax - DRMin > %u for key = 0x%"PRIx64,
			MAP_MAX_MIN_DELTA - 1, pItem->m_Key );
		MapConsoleDumpSubtree( pItem );
		ASSERT( FALSE );
		return( MAP_ROTNONE );
	}

	// We can never be out of balance on the same side
	// since that means a node below is out of balance.
	ASSERT( pItem->m_DLMax - pItem->m_DRMin <= MAP_MAX_MIN_DELTA );
	ASSERT( pItem->m_DRMax - pItem->m_DLMin <= MAP_MAX_MIN_DELTA );

	ASSERT( pItem->m_DLMax - pItem->m_DLMin < MAP_MAX_MIN_DELTA );
	ASSERT( pItem->m_DRMax - pItem->m_DRMin < MAP_MAX_MIN_DELTA );

#else
	// We can never be out of balance on the same side
	// since that means a node below is out of balance.
	ASSERT( pItem->m_DLMax - pItem->m_DRMin <= MAP_MAX_MIN_DELTA );
	ASSERT( pItem->m_DRMax - pItem->m_DLMin <= MAP_MAX_MIN_DELTA );

	ASSERT( pItem->m_DLMax - pItem->m_DLMin < MAP_MAX_MIN_DELTA );
	ASSERT( pItem->m_DRMax - pItem->m_DRMin < MAP_MAX_MIN_DELTA );
#endif

	if( pItem->m_DLMax - pItem->m_DRMin == MAP_MAX_MIN_DELTA )
	{
		// We are heavy left.
		// Determine if we need ROTR, ROTDR or RECURSIVE
		if(pItem->m_DRMax > MAP_RECURSION_THRESHOLD )
			return MAP_RECURSIVE;

		if( pItem->m_pLeft->m_DLMax > pItem->m_pLeft->m_DRMax )
			return MAP_ROTR;

		return MAP_ROTDR;
	}

	if( pItem->m_DRMax - pItem->m_DLMin == MAP_MAX_MIN_DELTA )
	{
		// We are heavy right.
		// Determine if we need ROTL, ROTDL or RECURSIVE
		if(pItem->m_DLMax > MAP_RECURSION_THRESHOLD )
			return MAP_RECURSIVE;

		if( pItem->m_pRight->m_DRMax > pItem->m_pRight->m_DLMax )
			return MAP_ROTL;

		return MAP_ROTDL;
	}

	return MAP_ROTNONE;
}

//////////////////////////////////////////////////////////////////////////////
//
// MapGet
//
// Description:
//  Retrieves the user object for the specified key.
//
// Inputs:
//  pMap - pointer to a MAP_RES
//  Key - 64-bit user key value to search for
//
// Outputs:
//  None
//
// Returns:
//  Pointer to the user item
//  NULL if not found
//
//////////////////////////////////////////////////////////////////////////////
void*
MapGet(
	IN const MAP_RES* const pMap,
	IN const uint64 Key )
{
	MAP_ITEM *pItem;
	
	ASSERT( pMap );

	pItem = MapRoot( pMap );

	while( pItem != NULL )
	{
		if( Key < pItem->m_Key )
			pItem = pItem->m_pLeft;			// too small
		else
			if( Key > pItem->m_Key )
				pItem = pItem->m_pRight;	// too big
			else
				return pItem->m_pObj;		// just right
	}

	// could not find a match
	return NULL;
}

//////////////////////////////////////////////////////////////////////////////
//
// MapItorInit
//
// Description:
//  Sets an iterator to the specified node of a tree.
//	This is an internal function and should not be called by user's of map.
//
// Inputs:
//  pItem - pointer to MAP_ITEM (may be NULL)
//  pItor - pointer to a MAP_ITERATOR
//
// Outputs:
//  None
//
// Returns:
//  Pointer to the input MAP_ITEM
//
//////////////////////////////////////////////////////////////////////////////
MAP_ITEM*
MapItorInit(
	IN MAP_ITEM* const pItem,
	IN MAP_ITERATOR * const pItor )
{
	ASSERT( pItor );
	pItor->pItem = pItem;
	return( pItem );
}


//////////////////////////////////////////////////////////////////////////////
//
// MapItorGet
//
// Description:
//  Set the specified iterator to the item with the specified key.
//  This internal function should not be called by users of MAP_RES.
//
// Inputs:
//  pMap			- pointer to a MAP_RES object
//  pItor		- pointer to a MAP_ITERATOR object
//  Key			- Key of new object to seek
//
// Outputs:
//  None
//
// Returns:
//  pItem of the object if found
//  NULL otherwise
//
//////////////////////////////////////////////////////////////////////////////
MAP_ITEM*
MapItorGet(
	IN const MAP_RES * const pMap,
	IN MAP_ITERATOR * const pItor,
	IN const uint64 Key )
{
	MAP_ITEM *pItem;

	ASSERT( pMap );
	ASSERT( pItor );

	pItem = MapRoot( pMap );

	while( pItem != NULL )
	{
		if( Key < pItem->m_Key )
			pItem = pItem->m_pLeft;			// too small
		else
			if( Key > pItem->m_Key )
				pItem = pItem->m_pRight;	// too big
			else
			{
				MapItorInit( pItem, pItor );
				return( pItem );		// just right
			}
	}

	// couldn't find an item with the specified key
	pItor->pItem = NULL;
	return NULL;
}


//////////////////////////////////////////////////////////////////////////////
//
// MapItorBalance
//
// Description:
//  Balances the tree at the node pointed to by the iterator.
//
// Inputs:
//  pItor		- pointer to a MAP_ITERATOR object
//
// Outputs:
//  None
//
// Returns:
//  TRUE if a rebalance occurred.
//  FALSE otherwise
//
//////////////////////////////////////////////////////////////////////////////
boolean
MapItorBalance(
	IN MAP_ITERATOR* const pItor )
{
	switch( MapItemCheckRot( pItor->pItem ) )
	{
	case MAP_ROTL:
		MapRotLeft( pItor->pItem );			// Fix +2 RR
		break;

	case MAP_ROTDL:
		MapDoubleLeft( pItor->pItem );		// Fix +2 RL or +2 R0
		break;

	case MAP_ROTDR:
		MapDoubleRight( pItor->pItem );		// Fix -2 LR or -2 L0
		break;

	case MAP_ROTR:
		MapRotRight( pItor->pItem );		// Fix -2 LL
		break;

	case MAP_RECURSIVE:
		// no rotation available capable for fixing this mess
		MapItorRecursiveBalance( pItor );
		break;

	default:
		// no rebalance necessary
		return( FALSE );
	}

	return( TRUE );
}

//////////////////////////////////////////////////////////////////////////////
//
// MapItorRecursiveBalance
//
// Description:
//  Balances the tree at the node pointed to by the iterator.
//  This algorithm recursively calls MapItorWalkBackAndBalance.
//  This function should be used for unbalanced nodes with
//  more children than can be accomodated with the simple rotation
//  primitives.
//  This internal function should not be called by users of MAP_RES.
//
// Inputs:
//  pItor		- pointer to a MAP_ITERATOR object
//  ppRoot		- pointer to a pointer to the node at the iterator
//
// Outputs:
//  None
//
// Returns:
//  None
//
//////////////////////////////////////////////////////////////////////////////
void
MapItorRecursiveBalance(
	IN MAP_ITERATOR* const pItor )
{
	MAP_ITEM *pOldRoot;
	MAP_ITEM *pNewRoot;
	MAP_ITEM *pInsertPoint;
	MAP_ITERATOR HeavyItor;
	MAP_ITERATOR LightItor;

	pOldRoot = pItor->pItem;
#if MAP_DEBUG > 1
	DbgOut("\nRecursively balancing at key = 0x%X", pOldRoot->m_Key );
	DbgOut("\nBefore:");
	MapConsoleDumpSubtree( pOldRoot );
#endif

	// Prepare some iterators for balancing the heavy and light subtrees.
	MapItorInit( pOldRoot, &HeavyItor );
	MapItorInit( pOldRoot, &LightItor );

	if( pOldRoot->m_DLMax - pOldRoot->m_DRMin == MAP_MAX_MIN_DELTA )
	{
		// We are heavy left.
		// The new root will be the rightmost node in the left subtree.
		// Traverse to the this new root and remove it from the subtree.
		MapItorLeft( &HeavyItor );
		pNewRoot = MapItorProbeRight( &HeavyItor );
		MapItorDeleteRightHalfLeaf( &HeavyItor );

		// install the new root
		MapItemExchange( pOldRoot, pNewRoot );
			
		// Now, insert the old root at the leftmost point in
		// the right subtree.
		MapItorRight( &LightItor );
		pInsertPoint = MapItorProbeLeft( &LightItor );
		MapItemInsertLeft( pOldRoot, pInsertPoint );
	}
	else
	{
		ASSERT( pOldRoot->m_DRMax - pOldRoot->m_DLMin == MAP_MAX_MIN_DELTA );
		// We are heavy right.
		// The new root will be the rightmost node in the left subtree.
		// Traverse to the this new root and remove it from the subtree.
		MapItorRight( &HeavyItor );
		pNewRoot = MapItorProbeLeft( &HeavyItor );
		MapItorDeleteLeftHalfLeaf( &HeavyItor );

		// install the new root
		MapItemExchange( pOldRoot, pNewRoot );

		// Now, insert the old root at the leftmost point in
		// the right subtree.
		MapItorLeft( &LightItor );
		pInsertPoint = MapItorProbeRight( &LightItor );
		MapItemInsertRight( pOldRoot, pInsertPoint );
	}

	// Now, the new tree is complete, but still unbalanced
#if MAP_DEBUG > 2
	DbgOut("\nAfter reposition, before heavy rebalance at key = 0x%X",
			pNewRoot->m_Key );
	DbgOut("\nAfter:");
	MapConsoleDumpSubtree( pNewRoot );
#endif

	MapItorWalkBackAndBalanceToItem( &HeavyItor, pNewRoot );
#if MAP_DEBUG > 2
	DbgOut("\nBefore light rebalance, after heavy rebalance at key = 0x%X",
			pNewRoot->m_Key );
	DbgOut("\nAfter:");
	MapConsoleDumpSubtree( pNewRoot );
#endif
	MapItorWalkBackAndBalanceToItem( &LightItor, pNewRoot );
	// don't balance above the new root!
	// the new root is the last node we need to leave in a 
	// balanced state
	// The function above leave LightItor at the new root
	MapUpdateDepthsNoCheck( pNewRoot );

	ASSERT( LightItor.pItem == pNewRoot );

	MapItorBalance( &LightItor );
#if MAP_DEBUG > 1
	DbgOut("\nDone recursively balancing at key = 0x%X", pNewRoot->m_Key );
	DbgOut("\nAfter:");
	MapConsoleDumpSubtree( pNewRoot );
#endif
}

//////////////////////////////////////////////////////////////////////////////
//
// MapItemExchange
//
// Description:
//  Replaces a MAP_ITEM with another MAP_ITEM
//
// Inputs:
//  pItor		- pointer to an initialized MAP_ITERATOR
//  pItem		- pointer to a new MAP_ITEM to insert
//
// Outputs:
//  None
//
// Returns:
//  None
//
//////////////////////////////////////////////////////////////////////////////
void
MapItemExchange(
	IN MAP_ITEM* const pOldItem,
	IN MAP_ITEM* const pNewItem )
{
	if ( NULL == pOldItem ) {
		ASSERT( FALSE );
		return;
	}

	if ( NULL == pNewItem ) {
		ASSERT( FALSE );
		return;
	}

	*MapItemGetPtrAbove( pOldItem ) = pNewItem;

	pNewItem->m_pLeft = pOldItem->m_pLeft;
	pNewItem->m_pRight = pOldItem->m_pRight;
	pNewItem->m_pUp = pOldItem->m_pUp;

	if( pNewItem->m_pLeft )
		pNewItem->m_pLeft->m_pUp = pNewItem;
	if( pNewItem->m_pRight )
		pNewItem->m_pRight->m_pUp = pNewItem;
}

//////////////////////////////////////////////////////////////////////////////
//
// MapItemInsertLeft
//
// Description:
//  Replaces a MAP_ITEM with another MAP_ITEM
//
// Inputs:
//
// Outputs:
//  None
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
MapItemInsertLeft(
	IN MAP_ITEM* const pInsertItem,
	IN MAP_ITEM* const pAtItem )
{
	if ( NULL == pAtItem ) {
		ASSERT( FALSE );
		return;
	}
	ASSERT( pAtItem->m_pLeft == NULL );

	pInsertItem->m_pRight = NULL;
	pInsertItem->m_pLeft = NULL;
	pInsertItem->m_pUp = pAtItem;
	pAtItem->m_pLeft = pInsertItem;
	MapItemResetBalances( pInsertItem );
}


//////////////////////////////////////////////////////////////////////////////
//
// MapItemInsertRight
//
// Description:
//  Replaces a MAP_ITEM with another MAP_ITEM
//
// Inputs:
//
// Outputs:
//  None
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
MapItemInsertRight(
	IN MAP_ITEM* const pInsertItem,
	IN MAP_ITEM* const pAtItem )
{
	if ( NULL == pAtItem ) {
		ASSERT( FALSE );
		return;
	}
	ASSERT( pAtItem->m_pRight == NULL );
	pInsertItem->m_pRight = NULL;
	pInsertItem->m_pLeft = NULL;
	pInsertItem->m_pUp = pAtItem;
	pAtItem->m_pRight = pInsertItem;
	MapItemResetBalances( pInsertItem );
}

//////////////////////////////////////////////////////////////////////////////
//
// MapItorInsert
//
// Description:
//  Insert an object into a MAP_RES.  An error is returned if the specified
//  key already exists in the map.
//
// Inputs:
//  pItor		- pointer to an initialized MAP_ITERATOR
//  pItem		- pointer to a new MAP_ITEM to insert
//
// Outputs:
//  None
//
// Returns:
//  FSUCCESS
//  FDUPLICATE - object with the specified key already exists in the map
//
//////////////////////////////////////////////////////////////////////////////
FSTATUS
MapItorInsert(
	IN MAP_ITERATOR* const pItor,
	IN MAP_ITEM* const pItem )
{
	uint64 Key;

	ASSERT( pItor );
	ASSERT( pItem );

	Key = pItem->m_Key;

	while( TRUE )
	{
		if( Key < pItor->pItem->m_Key )
		{
			// The new key is too small.
			// Traverse left if possible, otherwise, insert the new item
			// and leave the loop
			if( pItor->pItem->m_pLeft != NULL )
				MapItorLeft( pItor );
			else
			{
				MapItemInsertLeft( pItem, pItor->pItem );
				return FSUCCESS;
			}
		}
		else
		{
			if( Key > pItor->pItem->m_Key )
			{
				// The new key is too large - mirror image of process above
				if( pItor->pItem->m_pRight != NULL )
					MapItorRight( pItor );
				else
				{
					MapItemInsertRight( pItem, pItor->pItem );
					return FSUCCESS;
				}
			}
			else
				return( FDUPLICATE );	// Key already in the map!
		}
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// MapItorWalkBackAndBalance
//
// Description:
//  Walks the current iterator back to the root of the MAP_RES.
//  Along the way, recompute depth information and rebalance as necessary.
//
// Inputs:
//  pItor		- pointer to an initialized MAP_ITERATOR
//
// Outputs:
//  None
//
// Returns:
//  FSUCCESS
//  FDUPLICATE - object with the specified key already exists in the map
//
//////////////////////////////////////////////////////////////////////////////
void
MapItorWalkBackAndBalance(
	IN MAP_ITERATOR* const pItor )
{
	ASSERT( pItor );

	// special case if we're on a leaf
	// this prevents early short circuit in loop below
	if( MapIsLeaf( pItor->pItem ) )
	{
		MapItemResetBalances( pItor->pItem );
		MapItorUp( pItor );
	}

	do
	{
		// If the depth of a given node does not change
		// then none of the nodes above will change.
		// Thus, we can take a short-circuit exit here.

		MapUpdateDepthsNoCheck( pItor->pItem );

/*
		if( !MapUpdateDepths( pItor->pItem ) )
			return;
*/


		// balance as necessary
		// if a balance occurs, the new root is guaranteed to be balanced
		// so move the iterator up to prevent false negative on the
		// updatedepths call above
		if( MapItorBalance( pItor ) )
			MapItorUp( pItor );

	// go up one level and check if we're done
	} while( !MapItorAtRoot( MapItorUp( pItor ) ) );
}

//////////////////////////////////////////////////////////////////////////////
//
// MapItorWalkBackAndBalance
//
// Description:
//  Walks the current iterator back to the root of the MAP_RES.
//  Along the way, recompute depth information and rebalance as necessary.
//
// Inputs:
//  pItor		- pointer to an initialized MAP_ITERATOR
//
// Outputs:
//  None
//
// Returns:
//  FSUCCESS
//  FDUPLICATE - object with the specified key already exists in the map
//
//////////////////////////////////////////////////////////////////////////////
void
MapItorWalkBackAndBalanceToItem(
	IN MAP_ITERATOR* const pItor,
	IN MAP_ITEM* const pItem )
{
	ASSERT( pItor );
	ASSERT( pItor->pItem != pItem );

	// special case if we're on a leaf
	// this prevents early short circuit in loop below
	if( MapIsLeaf( pItor->pItem ) )
	{
		MapItemResetBalances( pItor->pItem );
		MapItorUp( pItor );
	}

	do
	{
		// If the depth of a given node does not change
		// then none of the nodes above will change.
		// Thus, we can take a short-circuit exit here.

/*
		if( !MapUpdateDepths( pItor->pItem ) )
			return;
*/

		MapUpdateDepthsNoCheck( pItor->pItem );

		// balance as necessary
		// if a balance occurs, the new root is guaranteed to be balanced
		// so move the iterator up to prevent false negative on the
		// updatedepths call above
		if( MapItorBalance( pItor ) )
			MapItorUp( pItor );

	// go up one level and check if we're done
	} while( MapItorUp( pItor )->pItem != pItem );
}


//////////////////////////////////////////////////////////////////////////////
//
// MapInsert
//
// Description:
//  Insert an object into the map.  An error is returned if the specified
//  key already exists in the map.
//
// Inputs:
//  pMap		- pointer to a MAP_RES object
//  Key			- Key of new object to insert
//  pObj		- pointer to user object to insert into the map
//
// Outputs:
//  None
//
// Returns:
//  FSUCCESS
//  FDUPLICATE - object with the specified key already exists in the map
//  FINSUFFICIENT_MEMORY
//
//////////////////////////////////////////////////////////////////////////////
FSTATUS
MapInsert(
	IN MAP_RES* const pMap,
	IN const uint64 Key,
	IN const void* const pObj )
{
	MAP_ITEM *pNew;
	MAP_ITERATOR Itor;
	FSTATUS Status;
	
	ASSERT( pMap );

	// acquire an empty map item for this new element
	// then, initialize its key and object
	pNew = (MAP_ITEM *)GrowPoolGet( &pMap->m_Pool );
	if( pNew == NULL )
		return FINSUFFICIENT_MEMORY;

	MapItemInit( pNew, Key, pObj );

	if( MapRoot( pMap ) == NULL )
	{
		// this is the first element in the tree
		MapSetRoot( pMap, pNew );
		pMap->m_Count++;
		return FSUCCESS;
	}

	MapItorInit( MapRoot( pMap ), &Itor );

	// After the insert, the iterator points to the parent
	// of the new node.
	Status = MapItorInsert( &Itor, pNew );
	if( Status != FSUCCESS )
	{
		// don't leak
		GrowPoolPut( &pMap->m_Pool, pNew );
		return Status;
	}

	// we successfully added an item to the tree
	pMap->m_Count++;

	//
	// We have added depth to this section of the tree.
	// Rebalance as necessary as we
	// retrace our path through the tree and update balances.
	//
	MapItorWalkBackAndBalance( &Itor );

	return FSUCCESS;
}

//////////////////////////////////////////////////////////////////////////////
//
// MapRemove
//
// Description:
//  Remove the node with the specified key.
//
// Inputs:
//  pMap		- pointer to a MAP_RES object
//  Key			- Key of new object to insert
//  pObj		- pointer to user object to insert into the map
//
// Outputs:
//  None
//
// Returns:
//  FSUCCESS
//  FNOT_FOUND - the specified key does not exist in the map
//
//////////////////////////////////////////////////////////////////////////////
FSTATUS
MapRemove(
	IN MAP_RES* const pMap,
	IN const uint64 Key )
{
	MAP_ITEM *pItem;
	MAP_ITEM *pSubsItem;
	MAP_ITERATOR RootItor;
	
	ASSERT( pMap );

	//
	// Phase One:
	// Seek the node with the specified key
	//
	pItem = MapItorGet( pMap, &RootItor, Key );

	if( pItem == NULL )
		return FNOT_FOUND;

	//
	// maybe we're lucky and the deleted node is a leaf
	//
	if( MapIsLeaf( pItem ) )
	{
		// yes, the node is a leaf
		MapItorDeleteLeaf( &RootItor );

		// return the deleted node to the pool
		GrowPoolPut( &pMap->m_Pool, pItem );
	}
	else
	{
		//
		// This item is not a leaf.
		// Use the DeleteToggle to decide from which side
		// we'll take the substitude node to replace the deleted node.
		// I'm speculating that toggling is more effecient that an
		// exhaustive analysis to determine which side is the best
		// choice from which to take the substitute leaf.  By choosing
		// arbitrarily, we have a better than 50% chance of being right,
		// since ties go in our favor.
		//
		if( pItem->m_pRight != NULL )
		{
			if( pItem->m_pLeft != NULL )
			{
				if( pMap->m_DelToggle )
				{
					// we're going to take node from the right subtree
					pMap->m_DelToggle = (boolean)(!pMap->m_DelToggle);
					MapItorRight( &RootItor );					// step right
					pSubsItem = MapItorProbeLeft( &RootItor );		// probe left
					MapItorDeleteLeftHalfLeaf( &RootItor );
				}
				else
				{
					// we're going to take node from the left subtree
					pMap->m_DelToggle = (boolean)(!pMap->m_DelToggle);
					MapItorLeft( &RootItor );					// step left
					pSubsItem = MapItorProbeRight( &RootItor );		// probe right
					MapItorDeleteRightHalfLeaf( &RootItor );
				}
			}
			else
			{
				// left subtree is NULL
				// we must take node from the right subtree
				pSubsItem = MapItorRight( &RootItor );
#if MAP_MAX_MIN_DELTA < 3
				MapItorDeleteLeaf( &RootItor );
#else
				pSubsItem = MapItorProbeLeft( &RootItor );		// probe left
				MapItorDeleteLeftHalfLeaf( &RootItor );
#endif
			}
		}
		else
		{
			// Right subtree is NULL
			// we must take node from the left subtree
			pSubsItem = MapItorLeft( &RootItor );
#if MAP_MAX_MIN_DELTA < 3
			MapItorDeleteLeaf( &RootItor );
#else
			pSubsItem = MapItorProbeRight( &RootItor );		// probe right
			MapItorDeleteRightHalfLeaf( &RootItor );
#endif
		}

		//
		// Phase Three:
		// We have identified the node to be deleted and the
		// node that will replace it.  Now, substitute the replacement
		// for the deleted item.
		// Note that the balance of the node may change from
		// the original node, but that gets taken care of later.
		//
		MapItemExchange( pItem, pSubsItem );

		if( RootItor.pItem == pItem )
			MapItorInit( pSubsItem, &RootItor );

		// return the deleted node to the pool
		GrowPoolPut( &pMap->m_Pool, pItem );
	}

	// we have successfully deleted the node
	if( --pMap->m_Count == 0 )
	{
		// there is nothing left in the tree!
		MapSetRoot( pMap, NULL );
		return FSUCCESS;
	}

	//
	// Phase Four:
	// The new tree is intact but possibly unbalanced.
	// We have removed depth from this section of the tree
	// Retrace our path through the tree and update balances.
	//
	MapItorWalkBackAndBalance( &RootItor );
	return FSUCCESS;
}

//////////////////////////////////////////////////////////////////////////////
//
// MapDoubleRight
//
// Description:
//  Double-rotate the subtree pointed to by *ppRoot to the right.
//  This transform can rebalance a -2 LR.
//  This internal function should not be called by users of MAP_RES.
//
// Inputs:
//  ppRoot - pointer to a pointer to a MAP_ITEM
//
// Outputs:
//  None
//
// Returns:
//  None
//
//////////////////////////////////////////////////////////////////////////////
void
MapDoubleRight(
	IN MAP_ITEM* const pItem )
{
	MAP_ITEM **ppRoot = MapItemGetPtrAbove( pItem );

	ASSERT( ppRoot );
	ASSERT( *ppRoot );
	ASSERT( (*ppRoot)->m_pLeft );
	ASSERT( (*ppRoot)->m_pLeft->m_pRight );

	// point the root to the new root
	*ppRoot = pItem->m_pLeft->m_pRight;
	(*ppRoot)->m_pUp = pItem->m_pUp;

	// setup old root's left item's right pointer with
	// the proper right subtree
	pItem->m_pLeft->m_pRight = (*ppRoot)->m_pLeft;
	if( (*ppRoot)->m_pLeft )
		(*ppRoot)->m_pLeft->m_pUp = pItem->m_pLeft;

	// setup the left pointer in the new root
	(*ppRoot)->m_pLeft = pItem->m_pLeft;
	if( pItem->m_pLeft->m_pUp )
		pItem->m_pLeft->m_pUp = *ppRoot;

	// setup old root's left pointer with the proper left subtree
	pItem->m_pLeft = (*ppRoot)->m_pRight;
	if( (*ppRoot)->m_pRight )
		(*ppRoot)->m_pRight->m_pUp = pItem;

	// setup the right pointer on the new root
	(*ppRoot)->m_pRight = pItem;
	pItem->m_pUp = *ppRoot;


	//
	// Now set the balances of the altered nodes
	// This must be done from the most distant nodes first,
	// then back towards the root.
	//
	MapUpdateDepthsNoCheck( (*ppRoot)->m_pLeft );
	MapUpdateDepthsNoCheck( (*ppRoot)->m_pRight );
	MapUpdateDepthsNoCheck( *ppRoot );
}

//////////////////////////////////////////////////////////////////////////////
//
// MapDoubleLeft
//
// Description:
//  Double rotate the subtree pointed to by *ppRoot to the Left.
//  This transform can rebalance a +2 RL.
//
// Inputs:
//  ppRoot - pointer to a pointer to a MAP_ITEM
//
// Outputs:
//  None
//
// Returns:
//  None
//
//////////////////////////////////////////////////////////////////////////////
void
MapDoubleLeft(
	IN MAP_ITEM* const pItem )
{
	MAP_ITEM **ppRoot = MapItemGetPtrAbove( pItem );

	ASSERT( ppRoot );
	ASSERT( *ppRoot );
	ASSERT( (*ppRoot)->m_pRight );
	ASSERT( (*ppRoot)->m_pRight->m_pLeft );

	// point the root to the new root
	// and record the old balance of the new root
	if ( NULL == pItem->m_pRight ) {
		ASSERT( FALSE );
		return;
	}
	*ppRoot = pItem->m_pRight->m_pLeft;
	(*ppRoot)->m_pUp = pItem->m_pUp;

	// setup old root's right item's left pointer with
	// the proper left subtree
	pItem->m_pRight->m_pLeft = (*ppRoot)->m_pRight;
	if( (*ppRoot)->m_pRight )
		(*ppRoot)->m_pRight->m_pUp = pItem->m_pRight;

	// setup the right pointer in the new root
	(*ppRoot)->m_pRight = pItem->m_pRight;
	if( pItem->m_pRight )
		pItem->m_pRight->m_pUp = *ppRoot;

	// setup old root's right pointer with the proper left subtree
	pItem->m_pRight = (*ppRoot)->m_pLeft;
	if( (*ppRoot)->m_pLeft )
		(*ppRoot)->m_pLeft->m_pUp = pItem;

	// setup the left pointer on the new root
	(*ppRoot)->m_pLeft = pItem;
	pItem->m_pUp = *ppRoot;

	//
	// Now set the balances of the altered nodes
	// This must be done from the most distant nodes first,
	// then back towards the root.
	//
	MapUpdateDepthsNoCheck( (*ppRoot)->m_pLeft );
	MapUpdateDepthsNoCheck( (*ppRoot)->m_pRight );
	MapUpdateDepthsNoCheck( *ppRoot );
}

//////////////////////////////////////////////////////////////////////////////
//
// MapUpdateDepthsNoCheck
//
// Description:
//  Updates the depth values of the specified item.  This algorithm assumes
//  the depths of all children of this item are already correct.
//  This internal function should not be called by users of MAP_RES.
//
// Inputs:
//  pItem - pointer to a MAP_ITEM
//
// Outputs:
//  None
//
// Returns:
//  None
//
//////////////////////////////////////////////////////////////////////////////
void
MapUpdateDepthsNoCheck(
	IN MAP_ITEM* const pItem )
{
	ASSERT( pItem );

	if( pItem->m_pLeft )
	{
		pItem->m_DLMin = (int8)(MIN( pItem->m_pLeft->m_DLMin, pItem->m_pLeft->m_DRMin ) + 1);
		pItem->m_DLMax = (int8)(MAX( pItem->m_pLeft->m_DLMax, pItem->m_pLeft->m_DRMax ) + 1);
	}
	else
	{
		pItem->m_DLMin = -1;
		pItem->m_DLMax = -1;
	}

	if( pItem->m_pRight )
	{
		pItem->m_DRMin = (int8)(MIN( pItem->m_pRight->m_DLMin, pItem->m_pRight->m_DRMin ) + 1);
		pItem->m_DRMax = (int8)(MAX( pItem->m_pRight->m_DLMax, pItem->m_pRight->m_DRMax ) + 1);
	}
	else
	{
		pItem->m_DRMin = -1;
		pItem->m_DRMax = -1;
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// MapUpdateDepths
//
// Description:
//  Updates the depth values of the specified item.  This algorithm assumes
//  the depths of all children of this item are already correct.
//  This internal function should not be called by users of MAP_RES.
//
// Inputs:
//  pItem - pointer to a MAP_ITEM
//
// Outputs:
//  None
//
// Returns:
//  TRUE if any of the depth values changed
//  FALSE otherwise
//
//////////////////////////////////////////////////////////////////////////////
boolean
MapUpdateDepths(
	IN MAP_ITEM* const pItem )
{
	int8 OldDLMin;
	int8 OldDLMax;
	int8 OldDRMin;
	int8 OldDRMax;

	ASSERT( pItem );
	ASSERT( !MapIsLeaf( pItem ) );

	// retain the old values for comparison
	OldDLMin = pItem->m_DLMin;
	OldDLMax = pItem->m_DLMax;
	OldDRMin = pItem->m_DRMin;
	OldDRMax = pItem->m_DRMax;

	MapUpdateDepthsNoCheck( pItem );

	// check for any change
	if( (pItem->m_DLMin != OldDLMin) || (pItem->m_DLMax != OldDLMax) ||
		(pItem->m_DRMin != OldDRMin) || (pItem->m_DRMax != OldDRMax) )
		return TRUE;

	// no change to the depth of this node
	return FALSE;
}


//////////////////////////////////////////////////////////////////////////////
//
// MapItorProbeLeft
//
// Description:
//  This function returns the leftmost child, given a starting iterator.
//  This is a helper function for traversing the tree.
//  Users of MAP_RES should not call this function.
//
// Input:
//  pItor - pointer to an iterator, possibly already in use
//
// Output
//  None
//
// Returns:
//  Pointer to the left-most item found in the subtree
//
//////////////////////////////////////////////////////////////////////////////
MAP_ITEM*
MapItorProbeLeft(
	IN MAP_ITERATOR* const pItor )
{
	MAP_ITEM *pItem = pItor->pItem;

	if( pItem == NULL )
		return NULL;

	//
	// Traverse as far to the left in the tree as we can go.
	// Record our path along the way.
	//
	while( pItem->m_pLeft != NULL )
		pItem = pItem->m_pLeft;

	pItor->pItem = pItem;
	return( pItem );
}

//////////////////////////////////////////////////////////////////////////////
//
// MapItorProbeRight
//
// Description:
//  This function returns the rightmost child, given a starting iterator.
//  This is a helper function for traversing the tree.
//  Users of MAP_RES should not call this function.
//
// Input:
//  pItor - pointer to an iterator, possibly already in use
//
// Output
//  None
//
// Returns:
//  Pointer to the right-most item found in the subtree
//
//////////////////////////////////////////////////////////////////////////////
MAP_ITEM* MapItorProbeRight( IN MAP_ITERATOR* const pItor )
{
	MAP_ITEM *pItem = pItor->pItem;

	if( pItem == NULL )
		return NULL;

	//
	// Traverse as far to the right in the tree as we can go.
	// Record our path along the way.
	//
	while( pItem->m_pRight != NULL )
		pItem = pItem->m_pRight;

	pItor->pItem = pItem;
	return( pItem );
}

//////////////////////////////////////////////////////////////////////////////
//
// MapHead
//
// Description:
//  Sets a map iterator to the first item in the map.  The first item in the
//  map is the item with the smallest key value.
//
// Inputs:
//  pMap - pointer to the map
//  pItor - pointer to a map iterator
//
// Outputs:
//  None
//
// Returns:
//  Pointer to the user item at the head of the map
//
//////////////////////////////////////////////////////////////////////////////
void*
MapHead(
	IN const MAP_RES* const pMap,
	IN MAP_ITERATOR* const pItor )
{
	MAP_ITEM *pItem;
	ASSERT( pMap );
	ASSERT( pItor );

	pItem = MapItorInit( MapRoot( pMap ), pItor );

	if( pItem )
	{
		pItem = MapItorProbeLeft( pItor );
		if( pItem )
			return pItem->m_pObj;
	}

	return NULL;
}

//////////////////////////////////////////////////////////////////////////////
//
// MapRemoveHead
//
// Description:
//  Removes the logical first element in the map.
//
// Inputs:
//  pMap - pointer to the map
//
// Outputs:
//  None
//
// Returns:
//  Pointer to the user item at the head of the map
//
//////////////////////////////////////////////////////////////////////////////
void*
MapRemoveHead(
	IN MAP_RES* const pMap )
{
	void *pObj;
	MAP_ITERATOR Itor;

	ASSERT( pMap );

	pObj = MapHead( pMap, &Itor );

	if( MapIsItorValid( &Itor ) )
		MapRemove( pMap, MapKey( &Itor ) );

	return pObj;
}

//////////////////////////////////////////////////////////////////////////////
//
// MapTail
//
// Description:
//  Sets a map iterator to the last item in the map.  The last item in the
//  map is the item with the largest key value.
//
// Inputs:
//  pMap - pointer to the map
//  pItor - pointer to a map iterator
//
// Outputs:
//  None
//
// Returns:
//  Pointer to the user item at the head of the map
//
//////////////////////////////////////////////////////////////////////////////
void*
MapTail(
	IN const MAP_RES* const pMap,
	IN MAP_ITERATOR* const pItor )
{
	MAP_ITEM *pItem;
	ASSERT( pMap );
	ASSERT( pItor );

	pItem = MapItorInit( MapRoot( pMap ), pItor );

	if( pItem )
	{
		pItem = MapItorProbeRight( pItor );
		if( pItem )
			return pItem->m_pObj;
	}

	return NULL;
}

//////////////////////////////////////////////////////////////////////////////
//
// MapRemoveTail
//
// Description:
//  Removes the last first element in the map.
//
// Inputs:
//  pMap - pointer to the map
//
// Outputs:
//  None
//
// Returns:
//  Pointer to the user item at the tail of the map
//
//////////////////////////////////////////////////////////////////////////////
void*
MapRemoveTail(
	IN MAP_RES* const pMap )
{
	void *pObj;
	MAP_ITERATOR Itor;

	ASSERT( pMap );

	pObj = MapTail( pMap, &Itor );

	if( MapIsItorValid( &Itor ) )
		MapRemove( pMap, MapKey( &Itor ) );

	return pObj;
}

//////////////////////////////////////////////////////////////////////////////
//
// MapItorNext
//
// Description:
//  Sets a map iterator to the next item in the map.  This is an internal
//  function that should not be called by users of map.
//
// Inputs:
//  pItor - pointer to a map iterator
//
// Outputs:
//  None
//
// Returns:
//  Pointer to the MAP_ITEM to which the iterator has moved
//
//////////////////////////////////////////////////////////////////////////////
MAP_ITEM*
MapItorNext(
	IN MAP_ITERATOR* const pItor )
{
	MAP_ITEM *pOldItem;
	//
	// Traverse the tree until we find the next logical node
	// Or we run out of nodes
	//
	if( pItor->pItem->m_pRight != NULL )
	{
		//
		// We must traverse the right subtree of this node
		//
		MapItorRight( pItor );
		return( MapItorProbeLeft( pItor ) );
	}
	else
	{
		// right pointer is null, so ascend
		do
		{
			pOldItem = pItor->pItem;
			MapItorUp( pItor );

		} while( pItor->pItem->m_pRight == pOldItem );

	}

	if( MapItorAtRoot( pItor ) )
		return( NULL );

	return( pItor->pItem );

}

//////////////////////////////////////////////////////////////////////////////
//
// MapItorPrev
//
// Description:
//  Sets a map iterator to the previous item in the map.  This is an internal
//  function that should not be called by users of map.
//
// Inputs:
//  pItor - pointer to a map iterator
//
// Outputs:
//  None
//
// Returns:
//  Pointer to the MAP_ITEM to which the iterator has moved
//
//////////////////////////////////////////////////////////////////////////////
MAP_ITEM* MapItorPrev( IN MAP_ITERATOR* const pItor )
{
	MAP_ITEM *pOldItem;
	//
	// Traverse the tree until we find the next logical node
	// Or we run out of nodes
	//
	if( pItor->pItem->m_pLeft != NULL )
	{
		//
		// We must traverse the left subtree of this node
		//
		MapItorLeft( pItor );
		return( MapItorProbeRight( pItor ) );
	}
	else
	{
		// left pointer is null, so ascend
		do
		{
			pOldItem = pItor->pItem;
			MapItorUp( pItor );

		} while( pItor->pItem->m_pLeft == pOldItem );

	}

	if( MapItorAtRoot( pItor ) )
		return( NULL );

	return( pItor->pItem );
}

//////////////////////////////////////////////////////////////////////////////
//
// MapDebugCheckDepth
//
// Description:
//  Returns FSUCCESS if the maximum and minimum depth of the tree vary by
//  no more than 1
//  This function should not be called by users of map.
//
// Inputs:
//  pMap - pointer to a MAP_RES
//
// Outputs:
//  None
//
// Returns:
//  FSUCCESS - depths do not vary by more than 1
//  FERROR otherwise
//
//////////////////////////////////////////////////////////////////////////////
/*
FSTATUS MapDebugCheckDepth( IN const MAP_RES * const pMap )
{
	MAP_ITERATOR Itor;
	MAP_ITEM *pItem;
	uint32 Min = 0xFFFFFFFF;
	uint32 Max = 0;
	uint32 Depth;

	MapItorInit( MapRoot( pMap ), &Itor );
	pItem = MapItorProbeLeft( &Itor );
	while( pItem != NULL )
	{
		if( (pItem->m_pLeft == NULL) || (pItem->m_pRight == NULL) )
		{
			// this is a leaf so mark the depth
			Depth = MapItorGetDepth( &Itor );
			if( Depth > Max )
				Max = Depth;
			if( Depth < Min )
				Min = Depth;
		}

		// verify the balance is legit
		if( MapItemCheckRot( pItem ) != MAP_ROTNONE )
		{
			DbgOut("\nBad tree!  Bad bal = %d for key 0x%X", MapItemCheckRot( pItem ),
				pItem->m_Key );
			return FERROR;
		}


		// try the next element
		pItem = MapItorNext( &Itor );
	}

	if( Max - Min < MAP_MAX_MIN_DELTA )
		return FSUCCESS;

	DbgOut("\nBad tree!  Min = %u, Max = %u", Min, Max );
	return FERROR;

}
*/

//////////////////////////////////////////////////////////////////////////////
//
// MapConsoleDump
//
// Description:
//  Prints a simple map the the current tree
//
// Inputs:
//  pMap		- pointer to a MAP_RES object
//  Key			- Key of new object to insert
//  pObj		- pointer to user object to insert into the map
//
// Outputs:
//  None
//
// Returns:
//  FSUCCESS
//  FNOT_FOUND - the specified key does not exist in the map
//
//////////////////////////////////////////////////////////////////////////////
void MapConsoleDump( IN const MAP_RES* const pMap )
{
	MapConsoleDumpSubtree( MapRoot( pMap ) );
}

//////////////////////////////////////////////////////////////////////////////
//
// MapConsoleDumpSubtree
//
// Description:
//  Prints a simple map the the current tree
//
// Inputs:
//  pMap		- pointer to a MAP_RES object
//  Key			- Key of new object to insert
//  pObj		- pointer to user object to insert into the map
//
// Outputs:
//  None
//
// Returns:
//  FSUCCESS
//  FNOT_FOUND - the specified key does not exist in the map
//
//////////////////////////////////////////////////////////////////////////////
void MapConsoleDumpSubtree( IN const MAP_ITEM* const pItem )
{
	uint8 Depth;

	Depth = 1;

	DbgOut("\n%u: ", Depth );
	while( MapConsoleDumpDepth( pItem, Depth, 1, 0 ) != 0 )
	{
		Depth++;
		DbgOut("\n%u: ", Depth);

		if( Depth > MAP_CONSOLE_DISPLAY_DEPTH_MAX )
		{
			DbgOut("More nodes beyond displayable depth...");
			break;
		}
	}

	DbgOut("\n\n");
}

uint32 MapConsoleDumpDepth(	IN const MAP_ITEM* const pItem,
						IN const uint8 ShowDepth,
						IN const uint8 CurrentDepth,
						IN uint32 Total )

{
	uint32 Width;
	uint32 FrontHalf;
	uint32 BackHalf;

	if( CurrentDepth == ShowDepth )
	{
		char Num[16];

		// display this node
		if( pItem != NULL )
		{
			if( CurrentDepth < 4 )
				// show balance in addition to key
				snprintf( Num, sizeof(Num) - 1, "%"PRIx64"[%d,%d,%d,%d]", pItem->m_Key,
					pItem->m_DLMin, pItem->m_DLMax,
					pItem->m_DRMin, pItem->m_DRMax );
			else
				snprintf( Num, sizeof(Num) - 1, "%"PRIx64, pItem->m_Key );

			Width = strlen( Num );
			// compensate if width is not an even number
			if( (Width % 2) == 0 )
				FrontHalf = BackHalf = Width / 2;
			else
			{
				if( Width == 1 )
				{
					FrontHalf = 0;
					BackHalf = 1;
				}
				else
				{
					FrontHalf = Width/2;
					BackHalf = Width/2 +1;
				}
			}

			MapConsolePad( ShowDepth, TRUE, FrontHalf, FALSE );
			DbgOut( "%s", Num );
			Total++;
			MapConsolePad( ShowDepth, FALSE, BackHalf, FALSE );
		}
		else
		{
			MapConsolePad( ShowDepth, TRUE, 0, TRUE );
			DbgOut("#");
			MapConsolePad( ShowDepth, FALSE, 1, TRUE );
		}

	}
	else
	{
		if( pItem != NULL )
		{
			// we're not deep enough, recurse left, then right
			Total += MapConsoleDumpDepth( pItem->m_pLeft, ShowDepth,
								(uint8)(CurrentDepth + 1), Total );

			Total += MapConsoleDumpDepth( pItem->m_pRight, ShowDepth,
								(uint8)(CurrentDepth + 1), Total );
		}
		else
			MapConsolePad( (uint8)(CurrentDepth - 1), FALSE, 0, TRUE );
	}

	return Total;
}

void MapConsolePad(	IN const uint8 ShowDepth,
					IN const boolean Leading,
					IN const uint32 FudgeFactor,
					IN const boolean Dashless )
{
	uint32 i;
	uint32 NumChars = 1<<(7 - ShowDepth);
	char DashChar;

	if( Dashless )
		DashChar = ' ';
	else
		DashChar = '-';

	// determine if we need to print leading or trailing '-'
	if( Leading )
	{
		// print leading '-'
		for( i = 0; i < NumChars - FudgeFactor; i++)
		{
			if( i < NumChars/2 )
				DbgOut(" ");
			else
				DbgOut("%c", DashChar);
		}
	}
	else
	{
		// print trailing '-'
		for( i = FudgeFactor; i < NumChars; i++)
		{
			if( i < NumChars/2 +1 )
				DbgOut("%c", DashChar);
			else
				DbgOut(" ");
		}
	}
}


