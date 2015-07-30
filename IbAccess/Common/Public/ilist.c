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

/*  NOTE: In order to support conditional inlining of ilist functions,
 *  the DEFINE_ILIST_FUNCS macro is defined *prior* to including the
 *  ilist.h header file. This file is the *only* source file that should
 *  be including ilist.h with the DEFINE_ILIST_FUNCS macro enabled.
 */
#define DEFINE_ILIST_FUNCS  1

#include "ilist.h"
#include "iobjmgr.h"
#include "imemory.h"

#define FREE_ITEM_GROW_SIZE		10

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//////////////													   ////////////
//////////////			 IMPLEMENTATION OF QUICK_LIST			   ////////////
//////////////													   ////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// QListInitState
// 
// Description:
//	This function initializes the state of the quicklist.
//	This function cannot fail.
// 
// Inputs:
//	pQuickList - Pointer to quicklist object
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void
QListInitState( 
	IN QUICK_LIST* const pQuickList )
{
	ASSERT( pQuickList );
	// a no-op - destroy can always be called 
}


///////////////////////////////////////////////////////////////////////////////
// QListInit
// 
// Description:
//	This function initializes the quicklist.
// 
// Inputs:
//	pQuickList - Pointer to quicklist object
// 
// Outputs:
//	None.
// 
// Returns:
//	TRUE on success
//	FALSE otherwise
// 
///////////////////////////////////////////////////////////////////////////////
boolean 
QListInit( 
	IN QUICK_LIST* const pQuickList )
{
	ASSERT( pQuickList );

	// Initialize the quick list data structure.
	QListRemoveAll( pQuickList );
	return TRUE;
}


///////////////////////////////////////////////////////////////////////////////
// QListDestroy
// 
// Description:
//	Destroys a list, discards any items on the list but does not free them
// 
// Inputs:
//	pQuickList - Pointer to quicklist object
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
QListDestroy( 
	IN QUICK_LIST* const pQuickList )
{
	ASSERT( pQuickList );
	// Remove all elements from the list.
	QListRemoveAll( pQuickList );
}


///////////////////////////////////////////////////////////////////////////////
// QListIsItemInList
// 
// Description:
//	Return true if the specified item is in the list
// 
// Inputs:
//	pQuickList	- Pointer to quicklist object
//	pListItem	- Pointer to the LIST_ITEM to find
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
boolean 
QListIsItemInList( 
	IN const QUICK_LIST* const pQuickList,
	IN const LIST_ITEM* const pListItem )
{
	const LIST_ITEM*	pTemp;

	ASSERT( pQuickList && pListItem );

	// Traverse looking for a match
	pTemp = QListHead( pQuickList );
	while( pTemp )
	{
		if( pTemp == pListItem )
			return( TRUE );

		pTemp = QListNext( pQuickList, pTemp );
	}

	return( FALSE );
}


///////////////////////////////////////////////////////////////////////////////
// QListAllocateAndInit
// 
// Description:
//	This function allocates memory for and initializes the state of the
//	quicklist.
// 
// Inputs:
//	IsPageable - 
//	Tag -
// 
// Outputs:
//	None.
// 
// Returns:
//	NULL on error, otherwise pointer to a ready to use List
// 
///////////////////////////////////////////////////////////////////////////////
QUICK_LIST* 
QListAllocateAndInit( 
	IN boolean IsPageable, 
	IN uint32 Tag )
{
	QUICK_LIST *pQuickList;

	pQuickList = (QUICK_LIST*)MemoryAllocateAndClear(sizeof(QUICK_LIST),
															 IsPageable, Tag);
	if (pQuickList != NULL)
	{
		QListInitState(pQuickList);
		if (! QListInit(pQuickList))
		{
			MemoryDeallocate(pQuickList);
			pQuickList = NULL;
		}
	}
	return pQuickList;
}

///////////////////////////////////////////////////////////////////////////////
// QListDeallocate
// 
// Description:
//	This function destroys the list and deallocates memory for the
//	quicklist.
//	It does not free items which were on the list
// 
// Inputs:
//	pQuickList - Pointer to quicklist object allocated by QListAllocateAndInit
//			if NULL, this function does nothing
// 
// Outputs:
//	None.
// 
// Returns:
// 
///////////////////////////////////////////////////////////////////////////////
void
QListDeallocate(
	IN QUICK_LIST* const pQuickList )
{
	if (pQuickList)
	{
		QListDestroy(pQuickList);
		MemoryDeallocate(pQuickList);
	}
}

///////////////////////////////////////////////////////////////////////////////
// QListRemoveAll
// 
// Description:
//	Removes all items in the specified list.
// 
// Inputs:
//	pQuickList - Pointer to quicklist object
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
QListRemoveAll( 
	IN QUICK_LIST* const pQuickList )
{
	ASSERT( pQuickList );

	// Clean up the pointers to indicate that the list is empty.
	pQuickList->m_Head.pNextItem = &pQuickList->m_Head;
	pQuickList->m_Head.pPrevItem = &pQuickList->m_Head;
	pQuickList->m_Count = 0;
}


///////////////////////////////////////////////////////////////////////////////
// QListInsertArrayHead
// 
// Description:
//	Insert an array of items to the head of a list.  The array pointer passed
//	into the function points to the LIST_ITEM in the first element of the
//	caller's element array.  There is no restriction on where the element is 
//	stored in the parent structure.
// 
// Inputs:
//	pQuickList	- Pointer to quicklist object.
//	pArray		- Pointer to LIST_ITEM of first element in array to add.
//	ItemCount	- number of items to add from the array.
//	ItemSize	- size of each item in the array.  This is the offset from
//				LIST_ITEM to LIST_ITEM.
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
QListInsertArrayHead( 
	IN QUICK_LIST* const pQuickList,
	IN LIST_ITEM* const pArray, 
	IN uint32 ItemCount,
	IN const uint32 ItemSize )
{
	LIST_ITEM	*pItem;

	ASSERT( pArray );
	ASSERT( ItemSize );
	ASSERT( ItemCount );

	// To add items from the array to the list in the same order as
	// the elements appear in the array, we add them starting with
	// the last one first.  Locate the last item.
	pItem = (LIST_ITEM*)((uint8 *)pArray + (ItemSize * (ItemCount - 1)));

	// Continue to add all items to the list.
	while( ItemCount-- )
	{
		QListInsertHead( pQuickList, pItem );

		// Get the next object to add to the list.
		pItem = (LIST_ITEM*)((uint8 *)pItem - ItemSize);
	}
}


///////////////////////////////////////////////////////////////////////////////
// QListInsertArrayTail
// 
// Description:
//	Insert an array of items to the tail of a list.  The array pointer passed
//	into the function points to the LIST_ITEM in the first element of the
//	caller's element array.  There is no restriction on where the element is 
//	stored in the parent structure.
// 
// Inputs:
//	pQuickList	- Pointer to quicklist object.
//	pArray		- Pointer to LIST_ITEM of first element in array to add.
//	ItemCount	- number of items to add from the array.
//	ItemSize	- size of each item in the array.  This is the offset from
//				LIST_ITEM to LIST_ITEM.
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
QListInsertArrayTail( 
	IN QUICK_LIST* const pQuickList,
	IN LIST_ITEM* const pArray, 
	IN uint32 ItemCount,
	IN const uint32 ItemSize )
{
	LIST_ITEM	*pItem;

	ASSERT( pArray && ItemSize && ItemCount );

	// Set the first item to add to the list.
	pItem = pArray;

	// Continue to add all items to the list.
	while ( ItemCount-- )
	{
		QListInsertTail( pQuickList, pItem );

		// Get the next object to add to the list.
		pItem = (LIST_ITEM*)((uint8*)pItem + ItemSize);
	}
}


///////////////////////////////////////////////////////////////////////////////
// QListInsertListHead
// 
// Description:
//	Add the items in a list to the head of another list.
// 
// Inputs:
//	pDestList	- Pointer to destination quicklist object.
//	pSrcList	- Pointer to quicklist to add.
// 
// Outputs:
// None
// 
// Returns:
// None
// 
///////////////////////////////////////////////////////////////////////////////
void 
QListInsertListHead( 
	IN QUICK_LIST* const pDestList,
	IN QUICK_LIST* const pSrcList )
{
	ASSERT( pDestList );
	ASSERT( pSrcList );

	// Is the src list empty?
	// We must have this check here for code below to work.
	if( QListIsEmpty( pSrcList ) )
		return;

	// Chain the destination list to the tail of the source list.
	pSrcList->m_Head.pPrevItem->pNextItem = pDestList->m_Head.pNextItem;
	pDestList->m_Head.pNextItem->pPrevItem = pSrcList->m_Head.pPrevItem;

	// Update the head of the destination list to the head of the source list.
	pDestList->m_Head.pNextItem = pSrcList->m_Head.pNextItem;
	pDestList->m_Head.pNextItem->pPrevItem = &pDestList->m_Head;

	// Adjust the count.
	pDestList->m_Count += pSrcList->m_Count;

	// Update source list to reflect being empty.
	QListRemoveAll( pSrcList );
}


///////////////////////////////////////////////////////////////////////////////
// QListInsertListTail
// 
// Description:
//	Add the items in a list to the tail of another list.
// 
// Inputs:
//	pDestList	- Pointer to destination quicklist object
//	pSrcList	- Pointer to quicklist to add
//
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
QListInsertListTail( 
	IN QUICK_LIST* const pDestList,
	IN QUICK_LIST* const pSrcList )
{
	ASSERT( pDestList );
	ASSERT( pSrcList );

	// Is the src list empty?
	// We must have this check here for code below to work.
	if( QListIsEmpty( pSrcList ) )
		return;

	// Chain the source list to the tail of the destination list.
	pDestList->m_Head.pPrevItem->pNextItem = pSrcList->m_Head.pNextItem;
	pSrcList->m_Head.pNextItem->pPrevItem = pDestList->m_Head.pPrevItem;

	// Update the tail of the destination list to the tail of the source list.
	pDestList->m_Head.pPrevItem = pSrcList->m_Head.pPrevItem;
	pDestList->m_Head.pPrevItem->pNextItem = &pDestList->m_Head;

	// Bump the count.
	pDestList->m_Count += pSrcList->m_Count;

	// Update source list to reflect being empty.
	QListRemoveAll( pSrcList );
}


///////////////////////////////////////////////////////////////////////////////
// QListGetItemAt (OBSOLETE - DO NOT USE)
// 
// Description:
//	Return the item at the specified offset into the list.  0 is the first item.
// 
// Inputs:
//	pQuickList	- Pointer to quicklist object
//	Index		- offset into list of item to return
// 
// Outputs:
//	None
// 
// Returns:
//	LIST_ITEM at specified index
//	NULL if item doesn't exist in the list
// 
///////////////////////////////////////////////////////////////////////////////
LIST_ITEM* 
QListGetItemAt( 
	IN const QUICK_LIST* const pQuickList,
	IN uint32 Index )
{
	LIST_ITEM		*pTemp;

	ASSERT( pQuickList );

	if( pQuickList->m_Count <= Index )
		return( NULL );

	// start at the front
	pTemp = QListHead( pQuickList );

	// if index is 0, we never enter the loop
	while( Index-- && pTemp )
		pTemp = QListNext( pQuickList, pTemp );

	return( pTemp );
}


///////////////////////////////////////////////////////////////////////////////
// QListFindFromHead
// 
// Description:
//	Use the specified function to search for an item starting at the head of
//	a list.  Returns the item if found, otherwise NULL.  If the user does not 
//	provide a compare function, then the pObject pointer in each LIST_ITEM is 
//	compared with the specified Context.
// 
// Inputs:
//	pQuickList	- Pointer to quicklist object
//	pfnFunc		- User supplied compare function
//	Context		- Context for compare function, or compare value if pfnFunc 
//				is NULL.
// 
// Outputs:
//	None
// 
// Returns:
//	Pointer to LIST_ITEM if found.
//	NULL otherwise
// 
///////////////////////////////////////////////////////////////////////////////
LIST_ITEM* 
QListFindFromHead( 
	IN const QUICK_LIST* const pQuickList,
	IN QLIST_FIND_FUNC pfnFunc OPTIONAL,
	IN void* const Context )
{
	LIST_ITEM	*pListItem;

	ASSERT( pQuickList );

	pListItem = QListHead( pQuickList );

	if( pfnFunc )
	{
		// The user provided a compare function
		while( pListItem )
		{
			if( pfnFunc( pListItem, Context ) )
				return( pListItem );
			pListItem = QListNext( pQuickList, pListItem );
		}
	}
	else
	{
		// The user didn't give us a compare function
		// Just compare context with the object pointer
		while( pListItem )
		{
			if( pListItem->pObject == Context )
				return( pListItem );
			pListItem = QListNext( pQuickList, pListItem );
		}
	}

	// No match
	return( NULL );
}


///////////////////////////////////////////////////////////////////////////////
// QListFindFromTail
// 
// Description:
//	Use the specified function to search for an item starting at the tail of
//	a list.  Returns the item if found, otherwise NULL.  If the user does not 
//	provide a compare function, then the pObject pointer in each LIST_ITEM is 
//	compared with the specified Context.
// 
// Inputs:
//	pQuickList	- Pointer to quicklist object
//	pfnFunc		- User supplied compare function
//	Context		- Context for compare function, or compare value if pfnFunc 
//				is NULL.
// 
// Outputs:
//	None
// 
// Returns:
//	Pointer to LIST_ITEM if found.
//	NULL otherwise
// 
///////////////////////////////////////////////////////////////////////////////
LIST_ITEM* 
QListFindFromTail( 
	IN const QUICK_LIST* const pQuickList,
	IN QLIST_FIND_FUNC pfnFunc OPTIONAL,
	IN void* const Context )
{
	LIST_ITEM	*pListItem;

	ASSERT( pQuickList );

	pListItem = QListTail( pQuickList );

	if( pfnFunc )
	{
		// The user provided a compare function
		while( pListItem )
		{
			if( pfnFunc( pListItem, Context ) )
				return( pListItem );
			pListItem = QListPrev( pQuickList, pListItem );
		}
	}
	else
	{
		// The user didn't give us a compare function
		// Just compare context with the object pointer
		while( pListItem )
		{
			if( pListItem->pObject == Context )
				return( pListItem );
			pListItem = QListPrev( pQuickList, pListItem );
		}
	}

	// No match
	return( NULL );
}


///////////////////////////////////////////////////////////////////////////////
// QListApplyFunc
// 
// Description:
//	Call the specified function for each item in the list.
// 
// Inputs:
//	pQuickList	- Pointer to quicklist object
//	pfnFunc		- User supplied function
//	Context		- Context for function
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
QListApplyFunc(
	IN const QUICK_LIST* const pQuickList, 
	IN QLIST_APPLY_FUNC pfnFunc,
	IN void* const Context )
{
	LIST_ITEM*	pListItem;

	// Note that context can have any arbitrary value.
	ASSERT( pQuickList );
	ASSERT( pfnFunc );

	pListItem = QListHead( pQuickList );
	while( pListItem )
	{
		pfnFunc( pListItem, Context );
		pListItem = QListNext( pQuickList, pListItem );
	}
}


///////////////////////////////////////////////////////////////////////////////
// QListMoveItems
// 
// Description:
//	Move items from one list to another based on a user supplied function.
//	Items are moved if Func() returns true.
//	Items are moved to the tail of the destination list.
// 
// Inputs:
//	pSrcList	- Pointer to source quicklist object
//	pDestList	- Pointer to destination quicklist object
//	pfnFunc		- User supplied function, returns true to move an item
//	Context		- Context for function
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
QListMoveItems( 
	IN QUICK_LIST* const pSrcList,
	IN QUICK_LIST* const pDestList,
	IN QLIST_FIND_FUNC pfnFunc,
	IN void* const Context )
{
	LIST_ITEM	*pCurrentItem, *pNextItem;

	ASSERT( pSrcList );
	ASSERT( pDestList );
	ASSERT( pfnFunc );

	pCurrentItem = QListHead( pSrcList );

	while( pCurrentItem )
	{
		// Before we do anything, get a pointer to the next item.
		pNextItem = QListNext( pSrcList, pCurrentItem );

		if( pfnFunc( pCurrentItem, Context ) )
		{
			// Move the item from one list to the other.
			QListRemoveItem( pSrcList, pCurrentItem );
			QListInsertTail( pDestList, pCurrentItem );
		}
		pCurrentItem = pNextItem;
	}
}





///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//////////////													   ////////////
//////////////			 IMPLEMENTATION OF LOCKED_QUICK_LIST	   ////////////
//////////////													   ////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// LQListInitState
// 
// Description:
//	This function initializes the state of the locked quicklist.
//	This function cannot fail.
// 
// Inputs:
//	pLQuickList - Pointer to locked quicklist object
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void
LQListInitState( 
	IN LOCKED_QUICK_LIST* const pLQuickList )
{
	ASSERT( pLQuickList );
	SpinLockInitState(&pLQuickList->m_Lock);
	QListInitState(&pLQuickList->m_List);
}


///////////////////////////////////////////////////////////////////////////////
// LQListInit
// 
// Description:
//	This function initializes the locked quicklist.
// 
// Inputs:
//	pLQuickList - Pointer to locked quicklist object
// 
// Outputs:
//	None.
// 
// Returns:
//	TRUE on success
//	FALSE otherwise
// 
///////////////////////////////////////////////////////////////////////////////
boolean 
LQListInit( 
	IN LOCKED_QUICK_LIST* const pLQuickList )
{
	boolean result;
	ASSERT( pLQuickList );

	if (! SpinLockInit(&pLQuickList->m_Lock))
	{
		return FALSE;
	}
	result = QListInit(&pLQuickList->m_List);
	if (! result)
	{
		SpinLockDestroy(&pLQuickList->m_Lock);
	}
	return result;
}


///////////////////////////////////////////////////////////////////////////////
// LQListDestroy
// 
// Description:
//	Destroys a locked list, discards any items on it (but does not free them)
//	The lock associated with the list is also destroyed.
// 
// Inputs:
//	pLQuickList - Pointer to locked quicklist object
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
LQListDestroy( 
	IN LOCKED_QUICK_LIST* const pLQuickList )
{
	ASSERT( pLQuickList );
	QListDestroy(&pLQuickList->m_List);
	SpinLockDestroy(&pLQuickList->m_Lock);
}


///////////////////////////////////////////////////////////////////////////////
// LQListIsItemInList
// 
// Description:
//	Return true if the specified item is in the list
// 
// Inputs:
//	pLQuickList	- Pointer to locked quicklist object
//	pListItem	- Pointer to the LIST_ITEM to find
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
boolean 
LQListIsItemInList( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN const LIST_ITEM* const pListItem )
{
	boolean result;

	ASSERT( pLQuickList && pListItem );
	SpinLockAcquire(&pLQuickList->m_Lock);
	result = QListIsItemInList(&pLQuickList->m_List, pListItem);
	SpinLockRelease(&pLQuickList->m_Lock);
	return result;
}


///////////////////////////////////////////////////////////////////////////////
// LQListAllocateAndInit
// 
// Description:
//	This function allocates memory for and initializes the state of the
//	quicklist.
// 
// Inputs:
//	IsPageable - 
//	Tag -
// 
// Outputs:
//	None.
// 
// Returns:
//	NULL on error, otherwise pointer to a ready to use List
// 
///////////////////////////////////////////////////////////////////////////////
LOCKED_QUICK_LIST* 
LQListAllocateAndInit( 
	IN boolean IsPageable, 
	IN uint32 Tag )
{
	LOCKED_QUICK_LIST *pLQuickList;

	pLQuickList = (LOCKED_QUICK_LIST*)MemoryAllocateAndClear(
									sizeof(LOCKED_QUICK_LIST), IsPageable, Tag);
	if (pLQuickList != NULL)
	{
		LQListInitState(pLQuickList);
		if (! LQListInit(pLQuickList))
		{
			MemoryDeallocate(pLQuickList);
			pLQuickList = NULL;
		}
	}
	return pLQuickList;
}

///////////////////////////////////////////////////////////////////////////////
// LQListDeallocate
// 
// Description:
//	This function destroys the list and deallocates memory for the
//	quicklist.
//	It does not free items which were on the list
// 
// Inputs:
//	pLQuickList - Pointer to quicklist object allocated by LQListAllocateAndInit
//			if NULL, this function does nothing
// 
// Outputs:
//	None.
// 
// Returns:
// 
///////////////////////////////////////////////////////////////////////////////
void
LQListDeallocate(
	IN LOCKED_QUICK_LIST* const pLQuickList )
{
	if (pLQuickList)
	{
		LQListDestroy(pLQuickList);
		MemoryDeallocate(pLQuickList);
	}
}

///////////////////////////////////////////////////////////////////////////////
// LQListRemoveAll
// 
// Description:
//	Removes all items in the specified list.
// 
// Inputs:
//	pLQuickList - Pointer to locked quicklist object
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
LQListRemoveAll( 
	IN LOCKED_QUICK_LIST* const pLQuickList )
{
	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	QListRemoveAll(&pLQuickList->m_List);
	SpinLockRelease(&pLQuickList->m_Lock);
}


///////////////////////////////////////////////////////////////////////////////
// LQListInsertArrayHead
// 
// Description:
//	Insert an array of items to the head of a list.  The array pointer passed
//	into the function points to the LIST_ITEM in the first element of the
//	caller's element array.  There is no restriction on where the element is 
//	stored in the parent structure.
// 
// Inputs:
//	pLQuickList	- Pointer to locked quicklist object.
//	pArray		- Pointer to LIST_ITEM of first element in array to add.
//	ItemCount	- number of items to add from the array.
//	ItemSize	- size of each item in the array.  This is the offset from
//				LIST_ITEM to LIST_ITEM.
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
LQListInsertArrayHead( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN LIST_ITEM* const pArray, 
	IN uint32 ItemCount,
	IN const uint32 ItemSize )
{
	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	QListInsertArrayHead(&pLQuickList->m_List, pArray, ItemCount, ItemSize);
	SpinLockRelease(&pLQuickList->m_Lock);
}


///////////////////////////////////////////////////////////////////////////////
// LQListInsertArrayTail
// 
// Description:
//	Insert an array of items to the tail of a list.  The array pointer passed
//	into the function points to the LIST_ITEM in the first element of the
//	caller's element array.  There is no restriction on where the element is 
//	stored in the parent structure.
// 
// Inputs:
//	pLQuickList	- Pointer to locked quicklist object.
//	pArray		- Pointer to LIST_ITEM of first element in array to add.
//	ItemCount	- number of items to add from the array.
//	ItemSize	- size of each item in the array.  This is the offset from
//				LIST_ITEM to LIST_ITEM.
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
LQListInsertArrayTail( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN LIST_ITEM* const pArray, 
	IN uint32 ItemCount,
	IN const uint32 ItemSize )
{
	ASSERT( pLQuickList);
	SpinLockAcquire(&pLQuickList->m_Lock);
	QListInsertArrayTail(&pLQuickList->m_List, pArray, ItemCount, ItemSize);
	SpinLockRelease(&pLQuickList->m_Lock);
}


///////////////////////////////////////////////////////////////////////////////
// LQListInsertListHead
// 
// Description:
//	Add the items in a list to the head of another list.
// 
// Inputs:
//	pDestList	- Pointer to destination locked quicklist object.
//	pSrcList	- Pointer to locked quicklist to add.
// 
// Outputs:
// None
// 
// Returns:
// None
// 
///////////////////////////////////////////////////////////////////////////////
void 
LQListInsertListHead( 
	IN LOCKED_QUICK_LIST* const pDestList,
	IN LOCKED_QUICK_LIST* const pSrcList )
{
	LIST_ITEM *pSrcHead;
	LIST_ITEM *pSrcTail;
	uint32 SrcCount;

	ASSERT( pDestList );
	ASSERT( pSrcList );

	// to avoid lock heirarchy issues, we never hold both locks at once
	// as such this is a specialized implementation rather than simple calls
	// to QListInsertListHead
	// This means there will be a brief instance where items in SrcList are in
	// neither list

	SpinLockAcquire(&pSrcList->m_Lock);
	// Is the src list empty?
	// We must have this check here for code below to work.
	if( QListIsEmpty( &pSrcList->m_List ) )
	{
		SpinLockRelease(&pSrcList->m_Lock);
		return;
	}
	pSrcHead = pSrcList->m_List.m_Head.pNextItem;
	pSrcTail = pSrcList->m_List.m_Head.pPrevItem;
	SrcCount = pSrcList->m_List.m_Count;
	// Update source list to reflect being empty.
	QListRemoveAll( &pSrcList->m_List );
	SpinLockRelease(&pSrcList->m_Lock);

	SpinLockAcquire(&pDestList->m_Lock);
	// Chain the destination list to the tail of the source list.
	pSrcTail->pNextItem = pDestList->m_List.m_Head.pNextItem;
	pDestList->m_List.m_Head.pNextItem->pPrevItem = pSrcTail;

	// Update the head of the destination list to the head of the source list.
	pDestList->m_List.m_Head.pNextItem = pSrcHead;
	pDestList->m_List.m_Head.pNextItem->pPrevItem = &pDestList->m_List.m_Head;

	// Adjust the count.
	pDestList->m_List.m_Count += SrcCount;
	SpinLockRelease(&pDestList->m_Lock);
}


///////////////////////////////////////////////////////////////////////////////
// LQListInsertListTail
// 
// Description:
//	Add the items in a list to the tail of another list.
// 
// Inputs:
//	pDestList	- Pointer to destination locked quicklist object
//	pSrcList	- Pointer to locked quicklist to add
//
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
LQListInsertListTail( 
	IN LOCKED_QUICK_LIST* const pDestList,
	IN LOCKED_QUICK_LIST* const pSrcList )
{
	LIST_ITEM *pSrcHead;
	LIST_ITEM *pSrcTail;
	uint32 SrcCount;

	ASSERT( pDestList );
	ASSERT( pSrcList );

	// to avoid lock heirarchy issues, we never hold both locks at once
	// as such this is a specialized implementation rather than simple calls
	// to QListInsertListHead
	// This means there will be a brief instance where items in SrcList are in
	// neither list

	SpinLockAcquire(&pSrcList->m_Lock);
	// Is the src list empty?
	// We must have this check here for code below to work.
	if( QListIsEmpty( &pSrcList->m_List ) )
	{
		SpinLockRelease(&pSrcList->m_Lock);
		return;
	}

	pSrcHead = pSrcList->m_List.m_Head.pNextItem;
	pSrcTail = pSrcList->m_List.m_Head.pPrevItem;
	SrcCount = pSrcList->m_List.m_Count;

	// Update source list to reflect being empty.
	QListRemoveAll( &pSrcList->m_List );
	SpinLockRelease(&pSrcList->m_Lock);

	SpinLockAcquire(&pDestList->m_Lock);
	// Chain the source list to the tail of the destination list.
	pDestList->m_List.m_Head.pPrevItem->pNextItem = pSrcHead;
	pSrcHead->pPrevItem = pDestList->m_List.m_Head.pPrevItem;

	// Update the tail of the destination list to the tail of the source list.
	pDestList->m_List.m_Head.pPrevItem = pSrcTail;
	pDestList->m_List.m_Head.pPrevItem->pNextItem = &pDestList->m_List.m_Head;

	// Bump the count.
	pDestList->m_List.m_Count += SrcCount;
	SpinLockRelease(&pDestList->m_Lock);
}


///////////////////////////////////////////////////////////////////////////////
// LQListGetItemAt (OBSOLETE - DO NOT USE)
// 
// Description:
//	Return the item at the specified offset into the list.  0 is the first item.
//	Must be used with care, as due to possible races, there is no guarantee
//	that in item is still on list after call completes
// 
// Inputs:
//	pLQuickList	- Pointer to locked quicklist object
//	Index		- offset into list of item to return
// 
// Outputs:
//	None
// 
// Returns:
//	LIST_ITEM at specified index
//	NULL if item doesn't exist in the list
// 
///////////////////////////////////////////////////////////////////////////////
LIST_ITEM* 
LQListGetItemAt( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN uint32 Index )
{
	LIST_ITEM		*pTemp;

	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	pTemp = QListGetItemAt(&pLQuickList->m_List, Index);
	SpinLockRelease(&pLQuickList->m_Lock);
	return pTemp;
}


///////////////////////////////////////////////////////////////////////////////
// LQListFindFromHead
// 
// Description:
//	Use the specified function to search for an item starting at the head of
//	a list.  Returns the item if found, otherwise NULL.  If the user does not 
//	provide a compare function, then the pObject pointer in each LIST_ITEM is 
//	compared with the specified Context.
//	List Lock will be held while function is called
//	Must be used with care, as due to possible races, there is no guarantee
//	that in item is still on list after call completes
// 
// Inputs:
//	pLQuickList	- Pointer to locked quicklist object
//	pfnFunc		- User supplied compare function
//	Context		- Context for compare function, or compare value if pfnFunc 
//				is NULL.
// 
// Outputs:
//	None
// 
// Returns:
//	Pointer to LIST_ITEM if found.
//	NULL otherwise
// 
///////////////////////////////////////////////////////////////////////////////
LIST_ITEM* 
LQListFindFromHead( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN QLIST_FIND_FUNC pfnFunc OPTIONAL,
	IN void* const Context )
{
	LIST_ITEM	*pListItem;

	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	pListItem = QListFindFromHead(&pLQuickList->m_List, pfnFunc, Context);
	SpinLockRelease(&pLQuickList->m_Lock);

	return pListItem;
}


///////////////////////////////////////////////////////////////////////////////
// LQListFindFromTail
// 
// Description:
//	Use the specified function to search for an item starting at the tail of
//	a list.  Returns the item if found, otherwise NULL.  If the user does not 
//	provide a compare function, then the pObject pointer in each LIST_ITEM is 
//	compared with the specified Context.
//	List Lock will be held while function is called
//	Must be used with care, as due to possible races, there is no guarantee
//	that in item is still on list after call completes
// 
// Inputs:
//	pLQuickList	- Pointer to locked quicklist object
//	pfnFunc		- User supplied compare function
//	Context		- Context for compare function, or compare value if pfnFunc 
//				is NULL.
// 
// Outputs:
//	None
// 
// Returns:
//	Pointer to LIST_ITEM if found.
//	NULL otherwise
// 
///////////////////////////////////////////////////////////////////////////////
LIST_ITEM* 
LQListFindFromTail( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN QLIST_FIND_FUNC pfnFunc OPTIONAL,
	IN void* const Context )
{
	LIST_ITEM	*pListItem;

	ASSERT( pLQuickList );

	SpinLockAcquire(&pLQuickList->m_Lock);
	pListItem = QListFindFromTail(&pLQuickList->m_List, pfnFunc, Context);
	SpinLockRelease(&pLQuickList->m_Lock);

	return pListItem;
}


///////////////////////////////////////////////////////////////////////////////
// LQListFindFromHeadAndRemove
// 
// Description:
//	Use the specified function to search for an item starting at the head of
//	a list.  Returns the item if found, otherwise NULL.  If the user does not 
//	provide a compare function, then the pObject pointer in each LIST_ITEM is 
//	compared with the specified Context.
//	If item is found in list it will be removed
//	List Lock will be held while function is called
// 
// Inputs:
//	pLQuickList	- Pointer to locked quicklist object
//	pfnFunc		- User supplied compare function
//	Context		- Context for compare function, or compare value if pfnFunc 
//				is NULL.
// 
// Outputs:
//	None
// 
// Returns:
//	Pointer to LIST_ITEM if found.
//	NULL otherwise
// 
///////////////////////////////////////////////////////////////////////////////
LIST_ITEM* 
LQListFindFromHeadAndRemove( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN QLIST_FIND_FUNC pfnFunc OPTIONAL,
	IN void* const Context )
{
	LIST_ITEM	*pListItem;

	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	pListItem = QListFindFromHead(&pLQuickList->m_List, pfnFunc, Context);
	if (pListItem)
	{
		QListRemoveItem(&pLQuickList->m_List, pListItem);
	}
	SpinLockRelease(&pLQuickList->m_Lock);

	return pListItem;
}


///////////////////////////////////////////////////////////////////////////////
// LQListFindFromTailAndRemove
// 
// Description:
//	Use the specified function to search for an item starting at the tail of
//	a list.  Returns the item if found, otherwise NULL.  If the user does not 
//	provide a compare function, then the pObject pointer in each LIST_ITEM is 
//	compared with the specified Context.
//	If item is found in list it will be removed
//	List Lock will be held while function is called
// 
// Inputs:
//	pLQuickList	- Pointer to locked quicklist object
//	pfnFunc		- User supplied compare function
//	Context		- Context for compare function, or compare value if pfnFunc 
//				is NULL.
// 
// Outputs:
//	None
// 
// Returns:
//	Pointer to LIST_ITEM if found.
//	NULL otherwise
// 
///////////////////////////////////////////////////////////////////////////////
LIST_ITEM* 
LQListFindFromTailAndRemove( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN QLIST_FIND_FUNC pfnFunc OPTIONAL,
	IN void* const Context )
{
	LIST_ITEM	*pListItem;

	ASSERT( pLQuickList );

	SpinLockAcquire(&pLQuickList->m_Lock);
	pListItem = QListFindFromTail(&pLQuickList->m_List, pfnFunc, Context);
	if (pListItem)
	{
		QListRemoveItem(&pLQuickList->m_List, pListItem);
	}
	SpinLockRelease(&pLQuickList->m_Lock);

	return pListItem;
}


///////////////////////////////////////////////////////////////////////////////
// LQListApplyFunc
// 
// Description:
//	Call the specified function for each item in the list.
//	List Lock will be held while function is called
// 
// Inputs:
//	pLQuickList	- Pointer to locked quicklist object
//	pfnFunc		- User supplied function
//	Context		- Context for function
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
LQListApplyFunc(
	IN LOCKED_QUICK_LIST* const pLQuickList, 
	IN QLIST_APPLY_FUNC pfnFunc,
	IN void* const Context )
{
	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	QListApplyFunc(&pLQuickList->m_List, pfnFunc, Context);
	SpinLockRelease(&pLQuickList->m_Lock);
}





///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//////////////													   ////////////
//////////////			 IMPLEMENTATION OF DLIST				   ////////////
//////////////													   ////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////



//
// Items placed on the list require the use of a LIST_ITEM.  A free pool
// of LIST_ITEMs is maintained to avoid allocating them with each insertion.
// To optimize performance when caching, the free list operates as a stack.
// 

//
// Attempt to grow the pool of available free LIST_ITEMs.
// 
void 
FreeItemPoolGrow( 
	IN DLIST* const pList )
{
	LIST_ITEM	*ListItemArray;

	// Allocate a new array of LIST_ITEMs.
	ListItemArray = (LIST_ITEM*)ObjMgrAllocate( pList->m_pObjMgr,
		sizeof( LIST_ITEM ) * pList->GrowItems );

	if( !ListItemArray )
		return;

	// Add the LIST_ITEMs to the free list.
	QListInsertArrayHead( &pList->m_FreeItemList, ListItemArray,
		pList->GrowItems, sizeof( LIST_ITEM ) );
}




///////////////////////////////////////////////////////////////////////////////
// ListInitState
// 
// Description:
//	Initialize the state of the list.  This function does not fail.
//	This should be the first function called for a list.
// 
// Inputs:
//	pList - Pointer to DLIST object
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
ListInitState( 
	IN DLIST* const pList )
{
	QListInitState( &pList->m_List );
	QListInitState( &pList->m_FreeItemList );
	pList->m_pObjMgr = NULL;
}


///////////////////////////////////////////////////////////////////////////////
// ListInit
// 
// Description:
//	Initialize the the list.  The list will always be able to store MinItems
//	items.
// 
// Inputs:
//	pList		- Pointer to DLIST object
//	MinItems	- The minimum number of items to support in the list.  If the
// 				number of items in the list exceeds this number,
// 				the list allocates more items automatically.
// 
// Outputs:
//	None
// 
// Returns:
//	TRUE on success
//	FALSE otherwise
// 
///////////////////////////////////////////////////////////////////////////////
boolean	
ListInit( 
	IN DLIST* const pList, 
	IN const uint32 MinItems )
{
	ASSERT( pList && MinItems );

	ListInitState( pList );
	if( !QListInit( &pList->m_FreeItemList ) )
		return FALSE;

	if( !QListInit( &pList->m_List ) )
	{
		ListDestroy( pList );
		return FALSE;
	}

	// Create and initialize the object manager.
	pList->m_pObjMgr = 
		(OBJECT_MGR*)MemoryAllocate( sizeof( OBJECT_MGR ), FALSE, 0 );
	if( !pList->m_pObjMgr ||
		!ObjMgrInit( pList->m_pObjMgr, FALSE ) )
	{
		ListDestroy( pList );
		return FALSE;
	}

	// Set the minimum number of items and grow the initial pool.
	pList->MinItems = MinItems;
	pList->GrowItems = MinItems;
	FreeItemPoolGrow( pList );
	if( !QListCount( &pList->m_FreeItemList ) )
	{
		// Could not create minimum number of LIST_ITEMs - abort.
		ListDestroy( pList );
		return FALSE;
	}

	// We will grow by MinItems/8 items at a time, with a minimum of 
	// FREE_ITEM_GROW_SIZE.
	pList->GrowItems >>= 3;
	if( pList->GrowItems < FREE_ITEM_GROW_SIZE )
		pList->GrowItems = FREE_ITEM_GROW_SIZE;

	return TRUE;
}


///////////////////////////////////////////////////////////////////////////////
// ListDestroy
// 
// Description:
//	Deallocate all internal memory used by the list.
//	This function does not fail.
// 
// Inputs:
//	pList - Pointer to DLIST object
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
ListDestroy( 
	IN DLIST* const pList )
{
	ASSERT( pList );

	QListDestroy( &pList->m_FreeItemList );
	QListDestroy( &pList->m_List );

	if( pList->m_pObjMgr )
	{
		ObjMgrDestroy( pList->m_pObjMgr );
		MemoryDeallocate( pList->m_pObjMgr );
		pList->m_pObjMgr = NULL;
	}
}


///////////////////////////////////////////////////////////////////////////////
// ListRemoveObject
// 
// Description:
//	Remove the item containing the specified user object from the list.
// 
// Inputs:
//	pList	- Pointer to DLIST object
//	pObject	- User object pointer (may be NULL)
// 
// Outputs:
//	None
// 
// Returns:
//	TRUE if the object was found and removed from the list
//	FALSE otherwise
// 
///////////////////////////////////////////////////////////////////////////////
boolean 
ListRemoveObject( 
	IN DLIST* const pList, 
	IN void* const pObject )
{
	LIST_ITEM	*pListItem;

	ASSERT( pList );

	// find the item in question
	pListItem = QListFindFromHead( &pList->m_List, NULL, pObject );
	if( pListItem )
	{
		// remove this item
		QListRemoveItem( &pList->m_List, pListItem );
		QListInsertHead( &pList->m_FreeItemList, pListItem );
		return TRUE;
	}
	return FALSE;
}


///////////////////////////////////////////////////////////////////////////////
// ListInsertArrayHead
// 
// Description:
//	Insert an array of new items at the head of the list.
// 
// Inputs:
//	pList		- Pointer to DLIST object
//	pArray		- Pointer to array
//	ItemCount	- Number of items in the array to insert
//	ItemSize	- Size (in bytes) of each item in the array
// 
// Outputs:
//	None
// 
// Returns:
//	TRUE on success
//	FALSE otherwise
// 
///////////////////////////////////////////////////////////////////////////////
boolean 
ListInsertArrayHead( 
	IN DLIST* const pList, 
	IN void* const pArray, 
	IN const uint32 ItemCount, 
	IN const uint32 ItemSize )
{
	void		*pObject;
	uint32		Count = ItemCount;

	ASSERT( pArray && ItemSize && ItemCount );

	// To add items from the array to the list in the same order as
	// the elements appear in the array, we add them starting with
	// the last one first.  Locate the last item.
	pObject = ((uchar*)pArray + (ItemSize * (ItemCount - 1)));

	// Continue to add all items to the list.
	while( Count-- )
	{
		if( !ListInsertHead( pList, pObject ) )
		{
			// Remove all items that have been inserted.
			while( Count++ < ItemCount )
				ListRemoveHead( pList );
			return FALSE;
		}

		// Get the next object to add to the list.
		pObject = ((uchar*)pObject - ItemSize);
	}

	return TRUE;
}


///////////////////////////////////////////////////////////////////////////////
// ListInsertArrayTail
// 
// Description:
//	Insert an array of new items at the tail of the list.
// 
// Inputs:
//	pList		- Pointer to DLIST object
//	pArray		- Pointer to array
//	ItemCount	- number of items in the array to insert
//	ItemSize	- Size (in bytes) of each item in the array
// 
// Outputs:
//	None
// 
// Returns:
//	TRUE on success
//	FALSE otherwise
// 
///////////////////////////////////////////////////////////////////////////////
boolean 
ListInsertArrayTail( 
	IN DLIST* const pList, 
	IN void* const pArray, 
	IN const uint32 ItemCount, 
	IN const uint32 ItemSize )
{
	void		*pObject;
	uint32		Count = ItemCount;

	ASSERT( pArray && ItemSize && ItemCount );

	// Set the first item to add to the list.
	pObject = pArray;

	// Continue to add all items to the list.
	while ( Count-- )
	{
		if( !ListInsertTail( pList, pObject ) )
		{
			// Remove all items that have been inserted.
			while( Count++ < ItemCount )
				ListRemoveTail( pList );
			return FALSE;
		}

		// Get the next object to add to the list.
		pObject = ((uchar*)pObject + ItemSize);
	}

	return TRUE;
}


//
// Returns the specified numbered item in a list.  If no item is found
// at that location, returns NULL.
// OBSOLETE!!! DO NOT USE!!!
// 
void* 
ListGetObjectAt( 
	IN const DLIST* const pList, 
	IN const uint32 Index )
{
	LIST_ITEM	*pListItem;

	ASSERT( pList );
	if( (pListItem = QListGetItemAt( &pList->m_List, Index )) )
		return pListItem->pObject;
	else
		return( NULL );
}


///////////////////////////////////////////////////////////////////////////////
// ListFindFromHead
// 
// Description:
//	Use the specified function to search for an object starting at the head of
//	a list.  Returns the item if found, otherwise NULL.  The list is not 
//	modified.
// 
// Inputs:
//	pList	- Pointer to DLIST object
//	pfnFunc	- Pointer to user's compare function
//	Context	- User context for compare function (may be NULL)
// 
// Outputs:
//	None
// 
// Returns:
//	Pointer to user object for which Func returned TRUE
//	NULL otherwise
// 
///////////////////////////////////////////////////////////////////////////////
void* 
ListFindFromHead( 
	IN const DLIST* const pList, 
	IN LIST_FIND_FUNC pfnFunc,
	IN void* const Context )
{
	LIST_ITEM*pListItem;

	// Note that context can have any arbitrary value.
	ASSERT( pList );
	ASSERT( pfnFunc );

	pListItem = QListHead( &pList->m_List );

	while( pListItem )
	{
		if( pfnFunc( pListItem->pObject, Context ) )
			return( pListItem->pObject );

		pListItem = QListNext( &pList->m_List, pListItem );
	}

	// no match
	return( NULL );
}


///////////////////////////////////////////////////////////////////////////////
// ListFindFromTail
// 
// Description:
//	Use the specified function to search for an object starting at the tail of
//	a list.  Returns the item if found, otherwise NULL.  The list is not 
//	modified.
// 
// Inputs:
//	pList	- Pointer to DLIST object
//	pfnFunc	- Pointer to user's compare function
//	Context	- User context for compare function (may be NULL)
// 
// Outputs:
//	None
// 
// Returns:
//	Pointer to user object for which Func returned TRUE
//	NULL otherwise
// 
///////////////////////////////////////////////////////////////////////////////
void* 
ListFindFromTail( 
	IN const DLIST* const pList, 
	IN LIST_FIND_FUNC pfnFunc,
	IN void* const Context )
{
	LIST_ITEM*	pListItem;

	// Note that context can have any arbitrary value.
	ASSERT( pList );
	ASSERT( pfnFunc );

	pListItem = QListTail( &pList->m_List );

	while( pListItem )
	{
		if( pfnFunc( pListItem->pObject, Context ) )
			return( pListItem->pObject );

		pListItem = QListPrev( &pList->m_List, pListItem );
	}

	// no match
	return( NULL );
}


///////////////////////////////////////////////////////////////////////////////
// ListApplyFunc
// 
// Description:
//	Calls the given function for all tiems in the list.  The list is not
//	modified.
// 
// Inputs:
//	pList	- Pointer to DLIST object
//	pfnFunc	- Pointer to user's compare function
//	Context - User context for compare function (may be NULL)
// 
// Outputs:
//	None
// 
// Returns:
//	None
// 
///////////////////////////////////////////////////////////////////////////////
void 
ListApplyFunc( 
	IN const DLIST* const pList, 
	IN LIST_APPLY_FUNC pfnFunc,
	IN void* const Context )
{
	LIST_ITEM*	pListItem;

	// Note that context can have any arbitrary value.
	ASSERT( pList );
	ASSERT( pfnFunc );

	pListItem = QListHead( &pList->m_List );

	while( pListItem )
	{
		pfnFunc( pListItem->pObject, Context );

		pListItem = QListNext( &pList->m_List, pListItem );
	}
}
