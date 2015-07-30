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

#ifndef _IBA_PUBLIC_ILIST_H_
#define _IBA_PUBLIC_ILIST_H_


#include "iba/public/datatypes.h"
#include "iba/public/ispinlock.h"

/*  Many of the simpler, more commonly used list functions are inlined
 *  to improve performance. That inlining confuses the gdb stack trace
 *  however. To disable inlining while debugging, do the following:
 *
 *  1) Change the INLINE_LIST macro (below) from 1 to 0
 *
 *  2) Uncomment the lines in the IbAccess/Darwin/exports file which
 *     correspond to the list functions which are no longer inlined.
 */
#define INLINE_ILIST    1

#if INLINE_ILIST
#define CONDITIONALLY_INLINE static __inline
#else
#define CONDITIONALLY_INLINE
#endif

/*  NOTE: In order to conditionally inline the list functions (as
 *  described above), this header file is also used to *define*
 *  (not just declare) the non-inlined functions in ilist.c when
 *  inlining is disabled. The *only* source file that should be
 *  enabling the DEFINE_ILIST_FUNCS macro is ilist.c.
 */

#ifdef __cplusplus
extern "C" {
#endif


/*
 * Generic list item structure.  Used by all lists to store objects.
 * The previous and next pointers are used internally, and should not
 * be accessed by users.
 */
typedef struct _LIST_ITEM
{
	struct _LIST_ITEM	*pNextItem;		/* DO NOT USE!! */
	struct _LIST_ITEM	*pPrevItem;		/* DO NOT USE!! */
	void				*pObject;		/* User's context */

} LIST_ITEM;

/* define an opaque iterator for the LIST functions */
typedef void *LIST_ITERATOR;

typedef void
(*QLIST_APPLY_FUNC)( 
	IN LIST_ITEM *pListItem, 
	IN void *Context );

typedef boolean
(*QLIST_FIND_FUNC)( 
	IN LIST_ITEM *pListItem, 
	IN void *Context );

typedef void
(*LIST_APPLY_FUNC)( 
	IN void *pObject, 
	IN void *Context );

typedef boolean
(*LIST_FIND_FUNC)( 
	IN void *pObject, 
	IN void *Context );



/****************************************************************************
 * Quick list object definition.  The quick list stores user supplied "list 
 * items" in a doubly linked list.  Operations on a quick list always succeed
 * since it does not allocate memory on the fly.  The user of the 
 * quick list is responsible for locking and unlocking it to prevent 
 * corruption.
 *
 ****************************************************************************/
typedef struct _QUICK_LIST
{
	/* Private data structure used by the quick list object. */
	/* Users should not access these variables directly. */
	LIST_ITEM	m_Head;			/* DO NOT USE!! */
	/* Number of items in the list. */
	uint32		m_Count;		/* DO NOT USE!! */

} QUICK_LIST;

/****************************************************************************
 * Locked Quick list object definition.  This is a Quick List with an
 * associated lock.
 ****************************************************************************/
typedef struct _LOCKED_QUICK_LIST
{
	/* Users may access these directly for functions which must walk the list */
	/* beware, the LQList functions provided below will acquire the lock */
	/* and should not be used while the list is locked by the user */
	/* this is not ideal, but iterators which work in conjunction with locks */
	/* can be quite complex so this more simplistic approach is taken */
	QUICK_LIST	m_List;
	SPIN_LOCK	m_Lock;
} LOCKED_QUICK_LIST;

/****************************************************************************
 * List object definition.  The list stores user supplied objects in a doubly 
 * linked list.  Operations on a list may require the allocation of memory,
 * and may therefore fail.  The user of the list is responsible for 
 * locking and unlocking it to prevent corruption.
 *
 ****************************************************************************/
typedef struct _DLIST
{
	/* Private data structure used by the list object. */
	/* Users should not access these variables directly. */
	/* List of free "list items" used to add objects to the list. */
	QUICK_LIST			m_List;			/* DO NOT USE!! */
	QUICK_LIST			m_FreeItemList;	/* DO NOT USE!! */

	/* The list dynamically adds list items as needed.  The object */
	/* manager is used to track their allocation.  This is a pointer */
	/* to the object manager to avoid circular include dependencies. */
	struct _OBJECT_MGR	*m_pObjMgr;		/* DO NOT USE!! */
	uint32				MinItems;		/* DO NOT USE!! */
	uint32				GrowItems;		/* DO NOT USE!! */

} DLIST;


/****************************************************************************
 * Queue object definition.  The queue stores user supplied objects in a doubly 
 * linked list.  A queue stores and returns objects in FIFO order.
 * Operations on a queue may require the allocation of memory,
 * and may therefore fail.  The user of the queue is responsible for 
 * locking and unlocking it to prevent corruption.
 *
 ****************************************************************************/
typedef DLIST	QUEUE;


/****************************************************************************
 * Stack object definition.  The stack stores user supplied objects in a doubly 
 * linked list.  A stack stores and returns objects in LIFO order.
 * Operations on a stack may require the allocation of memory, and may
 * therefore fail.  The user of the stack is responsible for locking and 
 * unlocking it to prevent corruption.
 *
 ****************************************************************************/
typedef DLIST	STACK;


/****************************************************************************
 * PrimitiveInsert
 * 
 * Description:
 *	Add a new item in front of the specified item.  This is a low level 
 *	function for use internally by the queuing routines.
 * 
 * Inputs:
 *	pListItem	- Pointer to LIST_ITEM to insert in front of
 *	pNewItem	- Pointer to LIST_ITEM to add
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
CONDITIONALLY_INLINE void 
PrimitiveInsert( 
	IN LIST_ITEM* const pListItem,
	IN LIST_ITEM* const pNewItem )
#if INLINE_ILIST || DEFINE_ILIST_FUNCS
{
	pNewItem->pNextItem = pListItem;
	pNewItem->pPrevItem = pListItem->pPrevItem;
	pListItem->pPrevItem = pNewItem;
	pNewItem->pPrevItem->pNextItem = pNewItem;
}
#else
;
#endif

/****************************************************************************
 * PrimitiveRemove
 * 
 * Description:
 *	Remove an item from a list.  This is a low level routine
 *	for use internally by the queuing routines.
 * 
 * Inputs:
 *	pListItem - Pointer to LIST_ITEM to remove
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
CONDITIONALLY_INLINE void 
PrimitiveRemove( 
	IN LIST_ITEM* const pListItem )
#if INLINE_ILIST || DEFINE_ILIST_FUNCS
{
	/* set the back pointer */
	pListItem->pNextItem->pPrevItem = pListItem->pPrevItem;
	/* set the next pointer */
	pListItem->pPrevItem->pNextItem = pListItem->pNextItem;

	/* NULL the link pointers to indicate that item is not on any list */
	pListItem->pNextItem = NULL;
	pListItem->pPrevItem = NULL;
}
#else
;
#endif

/****************************************************************************
 * ListItemIsInAList
 * 
 * Description:
 *  Examines a list item's link pointers to determine whether it is currently
 *  on a list. This test relies upon those pointers being initialized to NULL
 *  and on the Remove() function NULLing those pointers.
 * 
 * Inputs:
 *	pListItem	- Pointer to LIST_ITEM to be tested
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	TRUE if item is on a list
 *	FALSE if list item is not on a list
 * 
 ****************************************************************************/
static __inline boolean
ListItemIsInAList( 
	IN LIST_ITEM* const pListItem )
{
    ASSERT( pListItem );
    return (pListItem->pNextItem != NULL);
}

/****************************************************************************
 * ListItemInitState
 * 
 * Description:
 *  NULLs a list item's link pointers to indicate that it is currently not
 *  on any list.
 * 
 * Inputs:
 *	pListItem	- Pointer to LIST_ITEM to be initialized
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static __inline void
ListItemInitState( 
	IN LIST_ITEM* const pListItem )
{
    ASSERT( pListItem );

    /*  NULL the link pointers to indicate item is not on any list */
    pListItem->pNextItem = NULL;
    pListItem->pPrevItem = NULL;
}


/****************************************************************************
 ****************************************************************************
 ***********													   **********
 ***********			   DECLARATION OF QUICK_LIST			   **********
 ***********													   **********
 ****************************************************************************
 ****************************************************************************/


/*
 * Set the object pointer in a list item.
 */
static __inline void
QListSetObj(
	IN LIST_ITEM* const pListItem,
	IN void* const pObject )
{
	ASSERT( pListItem );
	pListItem->pObject = pObject;
}


/*
 * Get the object pointed to by a list item.
 */
static __inline void*
QListObj(
	IN const LIST_ITEM* const pListItem )
{
	ASSERT( pListItem );

	return pListItem->pObject;
}


/****************************************************************************
 * QListInitState
 * 
 * Description:
 *	This function initializes the state of the quicklist.
 *	This function cannot fail.
 * 
 * Inputs:
 *	pQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void 
QListInitState( 
	IN QUICK_LIST* const pQuickList );


/****************************************************************************
 * QListInit
 * 
 * Description:
 *	This function initializes the quicklist.
 * 
 * Inputs:
 *	pQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	TRUE on success
 *	FALSE otherwise
 * 
 ****************************************************************************/
boolean 
QListInit( 
	IN QUICK_LIST* const pQuickList );


/****************************************************************************
 * QListDestroy
 * 
 * Description:
 *	Destroys a list and discards any items on the list without freeing them
 * 
 * Inputs:
 *	pQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
QListDestroy( 
	IN QUICK_LIST* const pQuickList );


/****************************************************************************
 * QListAllocateAndInit
 * 
 * Description:
 *	This function allocates memory for and initializes the state of the
 *	quicklist.
 * 
 * Inputs:
 *	IsPageable - 
 *	Tag -
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	NULL on error, otherwise pointer to a ready to use List
 * 
 ****************************************************************************/
QUICK_LIST* 
QListAllocateAndInit( 
	IN boolean IsPageable, 
	IN uint32 Tag );


/****************************************************************************
 * QListDeallocate
 * 
 * Description:
 *	This function destroys the list and deallocates memory for the
 *	quicklist.
 *	It does not free items which were on the list
 * 
 * Inputs:
 *	pQuickList - Pointer to quicklist object allocated by QListAllocateAndInit
 *			if NULL, this function does nothing
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 * 
 ****************************************************************************/
void
QListDeallocate(
	IN QUICK_LIST* const pQuickList );


/****************************************************************************
 * QListIsEmpty
 * 
 * Description:
 *	return true if the list is empty
 * 
 * Inputs:
 *	pQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	True if the list is empty
 *	False otherwise
 * 
 ****************************************************************************/
static __inline boolean 
QListIsEmpty( 
	IN const QUICK_LIST* const pQuickList )
{
	return( (boolean)(pQuickList->m_Count == 0) );
}


/****************************************************************************
 * QListCount
 * 
 * Description:
 *	Returns the number of items in the list.  This call is not expensive,
 *	since the QList routines always maintain the count value.
 * 
 * Inputs:
 *	pSrcList - Pointer to source quicklist object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Number of items in the list
 * 
 ****************************************************************************/
static __inline uint32 
QListCount( 
	IN const QUICK_LIST* const pQuickList )
{
	ASSERT( pQuickList );
	return pQuickList->m_Count;
}


/****************************************************************************
 * QListIsItemInList
 * 
 * Description:
 *	Return true if the specified item is in the list
 * 
 * Inputs:
 *	pQuickList	- Pointer to quicklist object
 *	pListItem	- Pointer to the LIST_ITEM to find
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
boolean	
QListIsItemInList( 
	IN const QUICK_LIST* const pQuickList,
	IN const LIST_ITEM* const pListItem );


/****************************************************************************
 * QListInsertHead
 * 
 * Description:
 *	Add a new item to the head of the list.  The caller does not need to
 *	initialize the LIST_ITEM to be added.
 * 
 * Inputs:
 *	pQuickList	- Pointer to quicklist object
 *	pListItem	- Pointer to LIST_ITEM to add
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
CONDITIONALLY_INLINE void 
QListInsertHead( 
	IN QUICK_LIST* const pQuickList,
	IN LIST_ITEM* const pListItem )
#if INLINE_ILIST || DEFINE_ILIST_FUNCS
{
	ASSERT( pQuickList );
	ASSERT( pListItem );

#ifdef QLIST_PARANOID
	ASSERT( !QListIsItemInList( pQuickList, pListItem ) );
#endif

	/* put this guy at the front */
	PrimitiveInsert( pQuickList->m_Head.pNextItem, pListItem );

	/* bump the count */
	pQuickList->m_Count++;
}
#else
;
#endif


/****************************************************************************
 * QListInsertTail
 * 
 * Description:
 *	Add a new item to the tail of the list.  The caller does not need to
 *	initialize the LIST_ITEM to be added.
 * 
 * Inputs:
 *	pQuickList	- Pointer to quicklist object
 *	pListItem	- Pointer to LIST_ITEM to add
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
CONDITIONALLY_INLINE void 
QListInsertTail( 
	IN QUICK_LIST* const pQuickList,
	IN LIST_ITEM* const pListItem )
#if INLINE_ILIST || DEFINE_ILIST_FUNCS
{
	ASSERT( pQuickList );
	ASSERT( pListItem );

#ifdef QLIST_PARANOID
	ASSERT( !QListIsItemInList( pQuickList, pListItem ) );
#endif

	/* put this guy at the back */
	/* In other words, put the new element in front of the head */
	/* which is the same as being at the tail */
	PrimitiveInsert( &pQuickList->m_Head, pListItem );

	/* bump the count */
	pQuickList->m_Count++;
}
#else
;
#endif


/****************************************************************************
 * QListInsertPrev
 * 
 * Description:
 *	Insert an item before the specified item for the given list.
 * 
 * Inputs:
 *	pQuickList	- Pointer to quicklist object
 *	pListItem	- Pointer to item in the list
 *	pNewItem	- Pointer to item to insert before pListItem
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
CONDITIONALLY_INLINE void 
QListInsertPrev( 
	IN QUICK_LIST* const pQuickList,
	IN LIST_ITEM* const pListItem, 
	IN LIST_ITEM* const pNewItem )
#if INLINE_ILIST || DEFINE_ILIST_FUNCS
{
	/* sanity checks */
	ASSERT( pQuickList );
	ASSERT( pListItem );
	ASSERT( pNewItem );

#ifdef QLIST_PARANOID
	ASSERT( QListIsItemInList( pQuickList, pListItem ) );
	ASSERT( !QListIsItemInList( pQuickList, pNewItem ) );
#endif

	PrimitiveInsert( pListItem, pNewItem );

	pQuickList->m_Count++;
}
#else
;
#endif


/****************************************************************************
 * QListInsertNext
 * 
 * Description:
 *	Insert an item after the specified item for the given list.
 * 
 * Inputs:
 *	pQuickList	- Pointer to quicklist object
 *	pListItem	- Pointer to item in the list
 *	pNewItem	- Pointer to item to insert before pListItem
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
CONDITIONALLY_INLINE void 
QListInsertNext( 
	IN QUICK_LIST* const pQuickList,
	IN LIST_ITEM* const pListItem, 
	IN LIST_ITEM* const pNewItem )
#if INLINE_ILIST || DEFINE_ILIST_FUNCS
{
	/* sanity checks */
	ASSERT( pQuickList );
	ASSERT( pListItem );
	ASSERT( pNewItem );

#ifdef QLIST_PARANOID
	ASSERT( QListIsItemInList( pQuickList, pListItem ) );
	ASSERT( !QListIsItemInList( pQuickList, pNewItem ) );
#endif

	PrimitiveInsert( pListItem->pNextItem, pNewItem );

	pQuickList->m_Count++;
}
#else
;
#endif


/****************************************************************************
 * QListHead
 * 
 * Description:
 *	Retruns a pointer to the head of a list.  Does not alter the list.
 *	Returns NULL if the list is empty.
 * 
 * Inputs:
 *	pQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to head of list
 * 
 ****************************************************************************/
static __inline LIST_ITEM* 
QListHead( 
	IN const QUICK_LIST* const pQuickList )
{
	ASSERT( pQuickList );

	/* make sure the list isn't empty */
	if( QListIsEmpty( pQuickList ) )
		return( NULL );

	return( pQuickList->m_Head.pNextItem );
}


/****************************************************************************
 * QListTail
 * 
 * Description:
 *	Retruns a pointer to the last element of a list.  Does not alter the list.
 *	Returns NULL if the list is empty.
 * 
 * Inputs:
 *	pQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to tail of list
 * 
 ****************************************************************************/
static __inline LIST_ITEM* 
QListTail( 
	IN const QUICK_LIST* const pQuickList )
{
	ASSERT( pQuickList );

	/* make sure the list isn't empty */
	if( QListIsEmpty( pQuickList ) )
		return( NULL );

	return( pQuickList->m_Head.pPrevItem );
}


/****************************************************************************
 * QListPrev
 * 
 * Description:
 *	Return a pointer to the element in front of the specified element.
 *	Returns NULL if there is no element in front of the specified element.
 * 
 * 
 * Inputs:
 *	pQuickList	- Pointer to quicklist object
 *	pListItem	- Pointer to item in the list
 *	pNewItem	- Pointer to item to insert before pListItem
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static __inline LIST_ITEM*
QListPrev( 
	IN const QUICK_LIST* const pQuickList,
	IN const LIST_ITEM* const pListItem )
{
	/* Sanity checks */
	ASSERT( pQuickList );
	ASSERT( pListItem );

#ifdef QLIST_PARANOID
	ASSERT( QListIsItemInList( pQuickList, pListItem ) );
#endif

	/* Return the previous item, if any. */
	if( pListItem->pPrevItem != &pQuickList->m_Head )
		return( pListItem->pPrevItem );
	else
		return( NULL );
}


/****************************************************************************
 * QListNext
 * 
 * Description:
 *	Return a pointer to the element in back of the specified element.
 *	Returns NULL if there is no element in back of the specified element.
 * 
 * Inputs:
 *	pQuickList	- Pointer to quicklist object
 *	pListItem	- Pointer to item in the list
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static __inline LIST_ITEM*
QListNext( 
	IN const QUICK_LIST* const pQuickList,
	IN const LIST_ITEM* const pListItem )
{
	/* sanity checks */
	ASSERT( pQuickList );
	ASSERT( pListItem );

#ifdef QLIST_PARANOID
	ASSERT( QListIsItemInList( pQuickList, pListItem ) );
#endif

	/* Return the next item, if any. */
	if( pListItem->pNextItem == &pQuickList->m_Head )
		return( NULL );
	else
		return( pListItem->pNextItem );
}


/****************************************************************************
 * QListRemoveHead
 * 
 * Description:
 *	Remove an item from the head of a list.
 * 
 * Inputs:
 *	pQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to LIST_ITEM removed from the list
 * 
 ****************************************************************************/
CONDITIONALLY_INLINE LIST_ITEM* 
QListRemoveHead( 
	IN QUICK_LIST* const pQuickList )
#if INLINE_ILIST || DEFINE_ILIST_FUNCS
{
	LIST_ITEM	*pItem;

	ASSERT( pQuickList );

	/* make sure the list isn't empty */
	if( !pQuickList->m_Count )
		return( NULL );

	pItem = pQuickList->m_Head.pNextItem;
	PrimitiveRemove( pItem );

	pQuickList->m_Count--;

	return pItem;
}
#else
;
#endif


/****************************************************************************
 * QListRemoveTail
 * 
 * Description:
 *	Remove an item from the tail of a list.
 * 
 * Inputs:
 *	pQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to LIST_ITEM removed from the list
 * 
 ****************************************************************************/
CONDITIONALLY_INLINE LIST_ITEM* 
QListRemoveTail( 
	IN QUICK_LIST* const pQuickList )
#if INLINE_ILIST || DEFINE_ILIST_FUNCS
{
	LIST_ITEM	*pItem;

	ASSERT( pQuickList );

	/* make sure the list isn't empty */
	if( QListIsEmpty( pQuickList ) )
		return( NULL );

	pItem = pQuickList->m_Head.pPrevItem;
	PrimitiveRemove( pItem );

	pQuickList->m_Count--;

	return pItem;
}
#else
;
#endif


/****************************************************************************
 * QListRemoveItem
 * 
 * Description:
 * Removes an item from the specified list.
 * 
 * Inputs:
 * pQuickList	- Pointer to quicklist object
 * pListItem	- Pointer to the list item to remove
 * 
 * Outputs:
 * None
 * 
 * Returns:
 * None
 * 
 ****************************************************************************/
CONDITIONALLY_INLINE void 
QListRemoveItem( 
	IN QUICK_LIST* const pQuickList,
	IN LIST_ITEM* const pListItem )
#if INLINE_ILIST || DEFINE_ILIST_FUNCS
{
	ASSERT( pQuickList );
	ASSERT( pListItem  );

#ifdef QLIST_PARANOID
	ASSERT(	QListIsItemInList( pQuickList, pListItem ) );
#endif

	PrimitiveRemove( pListItem );

	pQuickList->m_Count--;
}
#else
;
#endif


/****************************************************************************
 * QListSafeRemoveItem
 * 
 * Description:
 * Removes an item from the specified list only if its in the list
 * 
 * Inputs:
 * pQuickList	- Pointer to quicklist object
 * pListItem	- Pointer to the list item to remove
 * 
 * Outputs:
 * None
 * 
 * Returns:
 * TRUE - was on list
 * FALSE - was not on list
 * 
 ****************************************************************************/
CONDITIONALLY_INLINE boolean 
QListSafeRemoveItem( 
	IN QUICK_LIST* const pQuickList,
	IN LIST_ITEM* const pListItem )
#if INLINE_ILIST || DEFINE_ILIST_FUNCS
{
	ASSERT( pQuickList );
	ASSERT( pListItem  );

	if(	QListIsItemInList( pQuickList, pListItem ) )
	{
		PrimitiveRemove( pListItem );
		pQuickList->m_Count--;
		return TRUE;
	}
	return FALSE;
}
#else
;
#endif


/****************************************************************************
 * QListRemoveAll
 * 
 * Description:
 *	Removes all items in the specified list.
 * 
 * Inputs:
 *	pQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
QListRemoveAll( 
	IN QUICK_LIST* const pQuickList );



/****************************************************************************
 * QListInsertArrayHead
 * 
 * Description:
 *	Insert an array of items to the head of a list.  The array pointer passed
 *	into the function points to the LIST_ITEM in the first element of the
 *	caller's element array.  There is no restriction on where the element is 
 *	stored in the parent structure.
 * 
 * Inputs:
 *	pQuickList	- Pointer to quicklist object.
 *	pArray		- Pointer to LIST_ITEM of first element in array to add.
 *	ItemCount	- Number of items to add from the array.
 *	ItemSize	- Size of each item in the array.  This is the offset from
 *				LIST_ITEM to LIST_ITEM.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
QListInsertArrayHead( 
	IN QUICK_LIST* const pQuickList,
	IN LIST_ITEM* const pArray,
	IN uint32 ItemCount,
	IN const uint32 ItemSize );


/****************************************************************************
 * QListInsertArrayTail
 * 
 * Description:
 *	Insert an array of items to the tail of a list.  The array pointer passed
 *	into the function points to the LIST_ITEM in the first element of the
 *	caller's element array.  There is no restriction on where the element is 
 *	stored in the parent structure.
 * 
 * Inputs:
 *	pQuickList	- Pointer to quicklist object.
 *	pArray		- Pointer to LIST_ITEM of first element in array to add.
 *	ItemCount	- Number of items to add from the array.
 *	ItemSize	- Size of each item in the array.  This is the offset from
 *				LIST_ITEM to LIST_ITEM.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
QListInsertArrayTail( 
		IN QUICK_LIST* const pQuickList,
		IN LIST_ITEM* const pArray,
		IN uint32 ItemCount,
		IN const uint32 ItemSize);


/****************************************************************************
 * QListInsertListHead
 * 
 * Description:
 *	Add the items in a list to the head of another list.
 * 
 * Inputs:
 *	pDestList	- Pointer to destination quicklist object.
 *	pSrcList	- Pointer to quicklist to add.
 * 
 * Outputs:
 * None
 * 
 * Returns:
 * None
 * 
 ****************************************************************************/
void 
QListInsertListHead( 
	IN QUICK_LIST* const pDestList,
	IN QUICK_LIST* const pSrcList );


/****************************************************************************
 * QListInsertListTail
 * 
 * Description:
 *	Add the items in a list to the tail of another list.
 * 
 * Inputs:
 *	pDestList	- Pointer to destination quicklist object
 *	pSrcList	- Pointer to quicklist to add
 *
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
QListInsertListTail( 
	IN QUICK_LIST * const pDestList,
	IN QUICK_LIST * const pSrcList );


/****************************************************************************
 * QListGetItemAt (OBSOLETE - DO NOT USE)
 * 
 * Description:
 *	Return the item at the specified offset into the list.  0 is the first item.
 * 
 * Inputs:
 *	pQuickList	- Pointer to quicklist object
 *	Index		- Offset into list of item to return
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	LIST_ITEM at specified index
 *	NULL if item doesn't exist in the list
 * 
 ****************************************************************************/
/**** OBSOLETE - DO NOT USE FOR NEW CODE!!! */
LIST_ITEM* 
QListGetItemAt( 
	IN const QUICK_LIST* const pQuickList,
	IN uint32 Index );

/**** Define for backward compatibility. */
/**** OBSOLETE - DO NOT USE FOR NEW CODE!!! */
#define QListFindItem QListFindFromHead


/****************************************************************************
 * QListFindFromHead
 * 
 * Description:
 *	Use the specified function to search for an item starting at the head of
 *	a list.  Returns the item if found, otherwise NULL.  If the user does not 
 *	provide a compare function, then the pObject pointer in each LIST_ITEM is 
 *	compared with the specified Context.
 * 
 * Inputs:
 *	pQuickList	- Pointer to quicklist object
 *	pfnFunc		- User supplied compare function
 *	Context		- Context for compare function, or compare value if pfnFunc 
 *				is NULL.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to LIST_ITEM if found.
 *	NULL otherwise
 * 
 ****************************************************************************/
LIST_ITEM* 
QListFindFromHead( 
	IN const QUICK_LIST* const pQuickList,
	IN QLIST_FIND_FUNC pfnFunc OPTIONAL,
	IN void* const Context );


/****************************************************************************
 * QListFindFromTail
 * 
 * Description:
 *	Use the specified function to search for an item starting at the tail of
 *	a list.  Returns the item if found, otherwise NULL.  If the user does not 
 *	provide a compare function, then the pObject pointer in each LIST_ITEM is 
 *	compared with the specified Context.
 * 
 * Inputs:
 *	pQuickList	- Pointer to quicklist object
 *	pfnFunc		- User supplied compare function
 *	Context		- Context for compare function, or compare value if pfnFunc 
 *				is NULL.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to LIST_ITEM if found.
 *	NULL otherwise
 * 
 ****************************************************************************/
LIST_ITEM* 
QListFindFromTail( 
	IN const QUICK_LIST* const pQuickList,
	IN QLIST_FIND_FUNC pfnFunc OPTIONAL,
	IN void* const Context );


/****************************************************************************
 * QListApplyFunc
 * 
 * Description:
 *	Call the specified function for each item in the list.
 * 
 * Inputs:
 *	pQuickList	- Pointer to quicklist object
 *	pfnFunc		- User supplied function
 *	Context		- Context for function
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
QListApplyFunc( 
	IN const QUICK_LIST* const pQuickList,
	IN QLIST_APPLY_FUNC pfnFunc,
	IN void* const Context );


/****************************************************************************
 * QListMoveItems
 * 
 * Description:
 *	Move items from one list to another based on a user supplied function.
 *	Items are moved if Func() returns true.
 *	Items are moved to the tail of the destination list.
 * 
 * Inputs:
 *	pSrcList	- Pointer to source quicklist object
 *	pDestList	- Pointer to destination quicklist object
 *	pfnFunc		- User supplied function, returns true to move an item
 *	Context		- Context for function
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
QListMoveItems( 
	IN QUICK_LIST* const pSrcList, 
	IN QUICK_LIST* const pDestList,
	IN QLIST_FIND_FUNC pfnFunc, 
	IN void* const Context );


/****************************************************************************
 ****************************************************************************
 ***********													   **********
 ***********			   DECLARATION OF LOCKED_QUICK_LIST		   **********
 ***********													   **********
 ****************************************************************************
 ****************************************************************************/


/****************************************************************************
 * LQListInitState
 * 
 * Description:
 *	This function initializes the state of the quicklist.
 *	This function cannot fail.
 * 
 * Inputs:
 *	pLQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void 
LQListInitState( 
	IN LOCKED_QUICK_LIST* const pLQuickList );


/****************************************************************************
 * LQListInit
 * 
 * Description:
 *	This function initializes the quicklist.
 * 
 * Inputs:
 *	pQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	TRUE on success
 *	FALSE otherwise
 * 
 ****************************************************************************/
boolean 
LQListInit( 
	IN LOCKED_QUICK_LIST* const pLQuickList );


/****************************************************************************
 * LQListDestroy
 * 
 * Description:
 *	Destroys a list and discards any items on the list without freeing them
 * 
 * Inputs:
 *	pLQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
LQListDestroy( 
	IN LOCKED_QUICK_LIST* const pLQuickList );

/****************************************************************************
 * LQListAllocateAndInit
 * 
 * Description:
 *	This function allocates memory for and initializes the state of the
 *	quicklist.
 * 
 * Inputs:
 *	IsPageable - 
 *	Tag -
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	NULL on error, otherwise pointer to a ready to use List
 * 
 ****************************************************************************/
LOCKED_QUICK_LIST* 
LQListAllocateAndInit( 
	IN boolean IsPageable, 
	IN uint32 Tag );

/****************************************************************************
 * LQListDeallocate
 * 
 * Description:
 *	This function destroys the list and deallocates memory for the
 *	quicklist.
 *	It does not free items which were on the list
 * 
 * Inputs:
 *	pLQuickList - Pointer to quicklist object allocated by LQListAllocateAndInit
 *			if NULL, this function does nothing
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 * 
 ****************************************************************************/
void
LQListDeallocate(
	IN LOCKED_QUICK_LIST* const pLQuickList );

/****************************************************************************
 * LQListIsEmpty
 * 
 * Description:
 *	return true if the list is empty
 * 
 * Inputs:
 *	pLQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	True if the list is empty
 *	False otherwise
 * 
 ****************************************************************************/
static __inline boolean 
LQListIsEmpty( 
	IN LOCKED_QUICK_LIST* const pLQuickList )
{
	boolean result;
	ASSERT(pLQuickList);
	SpinLockAcquire(&pLQuickList->m_Lock);
	result = QListIsEmpty(&pLQuickList->m_List);
	SpinLockRelease(&pLQuickList->m_Lock);
	return result;
}


/****************************************************************************
 * LQListCount
 * 
 * Description:
 *	Returns the number of items in the list.  This call is not expensive,
 *	since the QList routines always maintain the count value.
 * 
 * Inputs:
 *	pSrcList - Pointer to source quicklist object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Number of items in the list
 * 
 ****************************************************************************/
static __inline uint32 
LQListCount( 
	IN LOCKED_QUICK_LIST* const pLQuickList )
{
	uint32 result;

	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	result = QListCount(&pLQuickList->m_List);
	SpinLockRelease(&pLQuickList->m_Lock);
	return result;
}


/****************************************************************************
 * LQListIsItemInList
 * 
 * Description:
 *	Return true if the specified item is in the list
 * 
 * Inputs:
 *	pLQuickList	- Pointer to quicklist object
 *	pListItem	- Pointer to the LIST_ITEM to find
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
boolean	
LQListIsItemInList( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN const LIST_ITEM* const pListItem );


/****************************************************************************
 * LQListInsertHead
 * 
 * Description:
 *	Add a new item to the head of the list.  The caller does not need to
 *	initialize the LIST_ITEM to be added.
 * 
 * Inputs:
 *	pLQuickList	- Pointer to quicklist object
 *	pListItem	- Pointer to LIST_ITEM to add
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static __inline void 
LQListInsertHead( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN LIST_ITEM* const pListItem )
{
	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	QListInsertHead(&pLQuickList->m_List, pListItem);
	SpinLockRelease(&pLQuickList->m_Lock);
}


/****************************************************************************
 * LQListInsertTail
 * 
 * Description:
 *	Add a new item to the tail of the list.  The caller does not need to
 *	initialize the LIST_ITEM to be added.
 * 
 * Inputs:
 *	pLQuickList	- Pointer to quicklist object
 *	pListItem	- Pointer to LIST_ITEM to add
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static __inline void 
LQListInsertTail( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN LIST_ITEM* const pListItem )
{
	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	QListInsertTail(&pLQuickList->m_List, pListItem);
	SpinLockRelease(&pLQuickList->m_Lock);
}


/****************************************************************************
 * LQListInsertPrev
 * 
 * Description:
 *	Insert an item before the specified item for the given list.
 * 
 * Inputs:
 *	pLQuickList	- Pointer to quicklist object
 *	pListItem	- Pointer to item in the list
 *	pNewItem	- Pointer to item to insert before pListItem
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static __inline void 
LQListInsertPrev( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN LIST_ITEM* const pListItem, 
	IN LIST_ITEM* const pNewItem )
{
	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	QListInsertPrev(&pLQuickList->m_List, pListItem, pNewItem);
	SpinLockRelease(&pLQuickList->m_Lock);
}


/****************************************************************************
 * LQListInsertNext
 * 
 * Description:
 *	Insert an item after the specified item for the given list.
 * 
 * Inputs:
 *	pLQuickList	- Pointer to quicklist object
 *	pListItem	- Pointer to item in the list
 *	pNewItem	- Pointer to item to insert before pListItem
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static __inline void 
LQListInsertNext( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN LIST_ITEM* const pListItem, 
	IN LIST_ITEM* const pNewItem )
{
	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	QListInsertNext(&pLQuickList->m_List, pListItem, pNewItem);
	SpinLockRelease(&pLQuickList->m_Lock);
}


/* The following functions are not provided for LOCKED_QUICK_LIST since they
 * are typically used to iterate the list.
 * caller must SpinLockAcquire pLQuickList->m_Lock during iteration of list
 * then use QList functions against pLQuickList->m_List
 * QListHead, QListTail, QListPrev, QListNext
 *
 * In some cases (such as emptying a list), functions like LQListRemoveHead
 * and LQListRemoveTail may be a better choice
 *
 * For QListMoveItems, user should instead implement a higher level lock which
 * covers the 2 lists involved in which case neither list could be a
 * LOCKED_QUICK_LIST.  At present there is only 1 use of this function in TSL
 */

/****************************************************************************
 * LQListRemoveHead
 * 
 * Description:
 *	Remove an item from the head of a locked list.
 * 
 * Inputs:
 *	pLQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to LIST_ITEM removed from the list
 * 
 ****************************************************************************/
static __inline LIST_ITEM* 
LQListRemoveHead( 
	IN LOCKED_QUICK_LIST* const pLQuickList )
{
	LIST_ITEM	*pItem;

	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	pItem = QListRemoveHead(&pLQuickList->m_List);
	SpinLockRelease(&pLQuickList->m_Lock);
	return pItem;
}


/****************************************************************************
 * LQListRemoveTail
 * 
 * Description:
 *	Remove an item from the tail of a list.
 * 
 * Inputs:
 *	pLQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to LIST_ITEM removed from the list
 * 
 ****************************************************************************/
static __inline LIST_ITEM* 
LQListRemoveTail( 
	IN LOCKED_QUICK_LIST* const pLQuickList )
{
	LIST_ITEM	*pItem;

	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	pItem = QListRemoveTail(&pLQuickList->m_List);
	SpinLockRelease(&pLQuickList->m_Lock);

	return pItem;
}


/****************************************************************************
 * LQListRemoveItem
 * 
 * Description:
 * Removes an item from the specified list.
 * 
 * Inputs:
 * pLQuickList	- Pointer to quicklist object
 * pListItem	- Pointer to the list item to remove
 * 
 * Outputs:
 * None
 * 
 * Returns:
 * None
 * 
 ****************************************************************************/
static __inline void 
LQListRemoveItem( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN LIST_ITEM* const pListItem )
{
	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	QListRemoveItem(&pLQuickList->m_List, pListItem);
	SpinLockRelease(&pLQuickList->m_Lock);
}

/****************************************************************************
 * LQListSafeRemoveItem
 * 
 * Description:
 * Removes an item from the specified list only if its in the list
 * 
 * Inputs:
 * pLQuickList	- Pointer to quicklist object
 * pListItem	- Pointer to the list item to remove
 * 
 * Outputs:
 * None
 * 
 * Returns:
 * TRUE - was on list
 * FALSE - was not on list
 * 
 ****************************************************************************/
static __inline boolean 
LQListSafeRemoveItem( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN LIST_ITEM* const pListItem )
{
	boolean result;
	ASSERT( pLQuickList );
	SpinLockAcquire(&pLQuickList->m_Lock);
	result =QListSafeRemoveItem(&pLQuickList->m_List, pListItem);
	SpinLockRelease(&pLQuickList->m_Lock);
	return result;
}


/****************************************************************************
 * QListRemoveAll
 * 
 * Description:
 *	Removes all items in the specified list.
 * 
 * Inputs:
 *	pLQuickList - Pointer to quicklist object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
LQListRemoveAll( 
	IN LOCKED_QUICK_LIST* const pLQuickList );


/****************************************************************************
 * LQListInsertArrayHead
 * 
 * Description:
 *	Insert an array of items to the head of a list.  The array pointer passed
 *	into the function points to the LIST_ITEM in the first element of the
 *	caller's element array.  There is no restriction on where the element is 
 *	stored in the parent structure.
 * 
 * Inputs:
 *	pLQuickList	- Pointer to quicklist object.
 *	pArray		- Pointer to LIST_ITEM of first element in array to add.
 *	ItemCount	- Number of items to add from the array.
 *	ItemSize	- Size of each item in the array.  This is the offset from
 *				LIST_ITEM to LIST_ITEM.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
LQListInsertArrayHead( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN LIST_ITEM* const pArray,
	IN uint32 ItemCount,
	IN const uint32 ItemSize );


/****************************************************************************
 * LQListInsertArrayTail
 * 
 * Description:
 *	Insert an array of items to the tail of a list.  The array pointer passed
 *	into the function points to the LIST_ITEM in the first element of the
 *	caller's element array.  There is no restriction on where the element is 
 *	stored in the parent structure.
 * 
 * Inputs:
 *	pLQuickList	- Pointer to quicklist object.
 *	pArray		- Pointer to LIST_ITEM of first element in array to add.
 *	ItemCount	- Number of items to add from the array.
 *	ItemSize	- Size of each item in the array.  This is the offset from
 *				LIST_ITEM to LIST_ITEM.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
LQListInsertArrayTail( 
		IN LOCKED_QUICK_LIST* const pLQuickList,
		IN LIST_ITEM* const pArray,
		IN uint32 ItemCount,
		IN const uint32 ItemSize);


/****************************************************************************
 * LQListInsertListHead
 * 
 * Description:
 *	Add the items in a list to the head of another list.
 * 
 * Inputs:
 *	pDestList	- Pointer to destination quicklist object.
 *	pSrcList	- Pointer to quicklist to add.
 * 
 * Outputs:
 * None
 * 
 * Returns:
 * None
 * 
 ****************************************************************************/
void 
LQListInsertListHead( 
	IN LOCKED_QUICK_LIST* const pDestList,
	IN LOCKED_QUICK_LIST* const pSrcList );


/****************************************************************************
 * LQListInsertListTail
 * 
 * Description:
 *	Add the items in a list to the tail of another list.
 * 
 * Inputs:
 *	pDestList	- Pointer to destination quicklist object
 *	pSrcList	- Pointer to quicklist to add
 *
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
LQListInsertListTail( 
	IN LOCKED_QUICK_LIST * const pDestList,
	IN LOCKED_QUICK_LIST * const pSrcList );


/****************************************************************************
 * LQListGetItemAt (OBSOLETE - DO NOT USE)
 * 
 * Description:
 *	Return the item at the specified offset into the list.  0 is the first item.
 *	Must be used with care, as due to possible races, there is no guarantee
 *	that in item is still on list after call completes
 * 
 * Inputs:
 *	pLQuickList	- Pointer to quicklist object
 *	Index		- Offset into list of item to return
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	LIST_ITEM at specified index
 *	NULL if item doesn't exist in the list
 * 
 ****************************************************************************/
/**** OBSOLETE - DO NOT USE FOR NEW CODE!!! */
LIST_ITEM* 
LQListGetItemAt( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN uint32 Index );

/**** Define for backward compatibility. */
/**** OBSOLETE - DO NOT USE FOR NEW CODE!!! */
#define LQListFindItem LQListFindFromHead


/****************************************************************************
 * LQListFindFromHead
 * 
 * Description:
 *	Use the specified function to search for an item starting at the head of
 *	a list.  Returns the item if found, otherwise NULL.  If the user does not 
 *	provide a compare function, then the pObject pointer in each LIST_ITEM is 
 *	compared with the specified Context.
 *	List Lock will be held while function is called
 *	Must be used with care, as due to possible races, there is no guarantee
 *	that in item is still on list after call completes
 * 
 * Inputs:
 *	pLQuickList	- Pointer to quicklist object
 *	pfnFunc		- User supplied compare function
 *	Context		- Context for compare function, or compare value if pfnFunc 
 *				is NULL.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to LIST_ITEM if found.
 *	NULL otherwise
 * 
 ****************************************************************************/
LIST_ITEM* 
LQListFindFromHead( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN QLIST_FIND_FUNC pfnFunc OPTIONAL,
	IN void* const Context );


/****************************************************************************
 * LQListFindFromTail
 * 
 * Description:
 *	Use the specified function to search for an item starting at the tail of
 *	a list.  Returns the item if found, otherwise NULL.  If the user does not 
 *	provide a compare function, then the pObject pointer in each LIST_ITEM is 
 *	compared with the specified Context.
 *	List Lock will be held while function is called
 *	Must be used with care, as due to possible races, there is no guarantee
 *	that in item is still on list after call completes
 * 
 * Inputs:
 *	pLQuickList	- Pointer to quicklist object
 *	pfnFunc		- User supplied compare function
 *	Context		- Context for compare function, or compare value if pfnFunc 
 *				is NULL.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to LIST_ITEM if found.
 *	NULL otherwise
 * 
 ****************************************************************************/
LIST_ITEM* 
LQListFindFromTail( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN QLIST_FIND_FUNC pfnFunc OPTIONAL,
	IN void* const Context );


/****************************************************************************
 * LQListFindFromHeadAndRemove
 * 
 * Description:
 *	Use the specified function to search for an item starting at the head of
 *	a list.  Returns the item if found, otherwise NULL.  If the user does not 
 *	provide a compare function, then the pObject pointer in each LIST_ITEM is 
 *	compared with the specified Context.
 *	If item is found on list, it will be removed from the list
 *	List Lock will be held while function is called
 * 
 * Inputs:
 *	pLQuickList	- Pointer to quicklist object
 *	pfnFunc		- User supplied compare function
 *	Context		- Context for compare function, or compare value if pfnFunc 
 *				is NULL.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to LIST_ITEM if found.
 *	NULL otherwise
 * 
 ****************************************************************************/
LIST_ITEM* 
LQListFindFromHeadAndRemove( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN QLIST_FIND_FUNC pfnFunc OPTIONAL,
	IN void* const Context );


/****************************************************************************
 * LQListFindFromTailAndRemove
 * 
 * Description:
 *	Use the specified function to search for an item starting at the tail of
 *	a list.  Returns the item if found, otherwise NULL.  If the user does not 
 *	provide a compare function, then the pObject pointer in each LIST_ITEM is 
 *	compared with the specified Context.
 *	If item is found on list, it will be removed from the list
 *	List Lock will be held while function is called
 * 
 * Inputs:
 *	pLQuickList	- Pointer to quicklist object
 *	pfnFunc		- User supplied compare function
 *	Context		- Context for compare function, or compare value if pfnFunc 
 *				is NULL.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to LIST_ITEM if found.
 *	NULL otherwise
 * 
 ****************************************************************************/
LIST_ITEM* 
LQListFindFromTailAndRemove( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN QLIST_FIND_FUNC pfnFunc OPTIONAL,
	IN void* const Context );

/****************************************************************************
 * LQListApplyFunc
 * 
 * Description:
 *	Call the specified function for each item in the list.
 *	List Lock will be held while function is called
 * 
 * Inputs:
 *	pLQuickList	- Pointer to quicklist object
 *	pfnFunc		- User supplied function
 *	Context		- Context for function
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
LQListApplyFunc( 
	IN LOCKED_QUICK_LIST* const pLQuickList,
	IN QLIST_APPLY_FUNC pfnFunc,
	IN void* const Context );


/****************************************************************************
 ****************************************************************************
 ***********													   **********
 ***********			   DECLARATION OF DLIST					   **********
 ***********													   **********
 ****************************************************************************
 ****************************************************************************/


/* Private function to grow the pool if internal LIST_ITEMs.
 * Users should not call this function.
 */
void 
FreeItemPoolGrow( 
	IN DLIST* const pList );



/*
 * Private functions to retrieves or create a LIST_ITEMs for use with 
 * the list.
 */
static __inline LIST_ITEM* 
FreeItemGet( 
	IN DLIST* const pList )
{
	/* Check for the availability of "LIST_ITEMs" in the free list. */
	if( !QListCount( &pList->m_FreeItemList ) )
	{
		/* We're out. Create new items. */
		FreeItemPoolGrow( pList );
	}

	return QListRemoveHead( &pList->m_FreeItemList );
}


/****************************************************************************
 * ListInitState
 * 
 * Description:
 *	Initialize the state of the DLIST.  This function does not fail.
 *	This should be the first function called for a DLIST.
 * 
 * Inputs:
 *	pList - Pointer to DLIST object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
ListInitState( 
	IN DLIST* const pList );


/****************************************************************************
 * ListInit
 * 
 * Description:
 *	Initialize the the DLIST.  The list will always be able to store MinItems
 *	items.
 * 
 * Inputs:
 *	pList		- Pointer to DLIST object
 *	MinItems	- The minimum number of items to support in the DLIST.  If the
 * 				number of items in the DLIST exceeds this number,
 * 				the DLIST allocates more items automatically.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	TRUE on success
 *	FALSE otherwise
 * 
 ****************************************************************************/
boolean	
ListInit( 
	IN DLIST* const pList, 
	IN const uint32 MinItems );


/****************************************************************************
 * ListDestroy
 * 
 * Description:
 *	Deallocate all internal memory used by the DLIST.
 *	This function does not fail.
 * 
 * Inputs:
 *	pList - Pointer to DLIST object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
ListDestroy( 
	IN DLIST* const pList );


/****************************************************************************
 * ListIsEmpty
 * 
 * Description:
 *	Return true if the DLIST is empty.
 * 
 * Inputs:
 *	pList - Pointer to DLIST object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	TRUE if the DLIST is empty
 *	FALSE otherwise
 * 
 ****************************************************************************/
static __inline boolean 
ListIsEmpty( 
	IN const DLIST* const pList )
{
	ASSERT( pList );
	return( QListIsEmpty( &pList->m_List ) );
}


/****************************************************************************
 * ListCount
 * 
 * Description:
 *	Returns the number of items in the DLIST.
 * 
 * Inputs:
 *	pList - Pointer to DLIST object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Number of items in the list.
 * 
 ****************************************************************************/
static __inline uint32
ListCount( 
	IN const DLIST* const pList )
{
	ASSERT( pList );
	return QListCount( &pList->m_List );
}




/****************************************************************************
 * ListInsertHead
 * 
 * Description:
 *	Insert a new item at the head of the list.
 * 
 * Inputs:
 *	pList	- Pointer to DLIST object
 *	pObject	- Pointer to user's object to insert into the list
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	TRUE on success
 *	FALSE otherwise
 * 
 ****************************************************************************/
static __inline boolean 
ListInsertHead( 
	IN DLIST* const pList, 
	IN void* const pObject )
{
	LIST_ITEM	*pListItem;

	ASSERT( pList);
	/* Get a LIST_ITEM to add to the list. */
	if( (pListItem = FreeItemGet( pList )) == NULL )
		return FALSE;

	pListItem->pObject = pObject;
	QListInsertHead( &pList->m_List, pListItem );
	return TRUE;
}


/****************************************************************************
 * ListInsertTail
 * 
 * Description:
 *	Insert a new item at the tail of the list.
 * 
 * Inputs:
 *	pList	- Pointer to DLIST object
 *	pObject	- Pointer to user's object to insert into the list
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	TRUE on success
 *	FALSE otherwise
 * 
 ****************************************************************************/
static __inline boolean 
ListInsertTail( 
	IN DLIST* const pList, 
	IN void* const pObject )
{
	LIST_ITEM	*pListItem;

	ASSERT( pList );
	/* Get a LIST_ITEM to add to the list. */
	if( (pListItem = FreeItemGet( pList )) == NULL )
		return FALSE;

	pListItem->pObject = pObject;
	QListInsertTail( &pList->m_List, pListItem );
	return TRUE;
}


/****************************************************************************
 * ListInsertPrev
 * 
 * Description:
 *	Insert a new item before the specified item for the given list.
 * 
 * Inputs:
 *	pList		- Pointer to list object
 *	Iterator	- LIST_ITERATOR to item in the list
 *	pObject		- Pointer to user's object to insert into the list
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	TRUE on success
 *	FALSE otherwise
 * 
 ****************************************************************************/
static __inline boolean 
ListInsertPrev(
	IN DLIST* const pList,
	IN LIST_ITERATOR const Iterator, 
	IN void* const pObject )
{
	LIST_ITEM	*pNewItem;

	ASSERT( pList);
	/* Get a LIST_ITEM to add to the list. */
	if( (pNewItem = FreeItemGet( pList )) == NULL )
		return FALSE;

	pNewItem->pObject = pObject;
	QListInsertPrev( &pList->m_List,
			(LIST_ITEM*)Iterator , pNewItem );
	return TRUE;
}


/****************************************************************************
 * ListInsertNext
 * 
 * Description:
 *	Insert a new item after the specified item for the given list.
 * 
 * Inputs:
 *	pList		- Pointer to DLIST object
 *	Iterator	- LIST_ITERATOR to item in the list.
 *	pObject		- Pointer to user's object to insert into the list
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	TRUE on success
 *	FALSE otherwise
 * 
 ****************************************************************************/
static __inline boolean 
ListInsertNext( 
	IN DLIST* const pList,
	IN LIST_ITERATOR const Iterator, 
	IN void* const pObject )
{
	LIST_ITEM	*pNewItem;

	ASSERT( pList);
	/* Get a LIST_ITEM to add to the list. */
	if( (pNewItem = FreeItemGet( pList )) == NULL )
		return FALSE;

	pNewItem->pObject = pObject;
	/* we have to cast away const here, since we are modifying */
	/* the LIST_ITEM */
	QListInsertNext( &pList->m_List,
				(LIST_ITEM*)Iterator , pNewItem );
	return TRUE;
}


/****************************************************************************
 * ListObj
 * 
 * Description:
 *	Return a pointer to the list_elements associated object
 * 
 * Inputs:
 *	pListItem - pointer to LIST_ITEM
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	pointer to user object associated with this list_item
 * 
 ****************************************************************************/
static __inline void*
ListObj( 
	IN LIST_ITERATOR const Iterator )
{
	return( ((LIST_ITEM*)Iterator)->pObject );
}


/****************************************************************************
 * ListHead
 * 
 * Description:
 *	Return a LIST_ITERATOR to the first item in the DLIST.
 * 
 *	The LIST_ITERATOR is opaque to the user.  To get the user object
 *	at this element, use ListObj().
 * 
 * Inputs:
 *	pList		- Pointer to DLIST object
 *	ListItem	- LIST_ITERATOR to item in the list
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	LIST_ITERATOR to the head of the DLIST
 *	NULL if the DLIST is empty
 * 
 ****************************************************************************/
static __inline LIST_ITERATOR 
ListHead( 
	IN const DLIST* const pList )
{
	ASSERT( pList );
	return( QListHead( &pList->m_List ) );
}


/****************************************************************************
 * ListTail
 * 
 * Description:
 *	Return a LIST_ITERATOR to the last item in the DLIST.
 * 
 *	The LIST_ITERATOR is opaque to the user.  To get the user object
 *	at this element, use ListObj().
 * 
 * Inputs:
 *	pList		- Pointer to DLIST object
 *	ListItem	- LIST_ITERATOR to item in the list
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	LIST_ITERATOR to the last item of the DLIST
 *	NULL if the DLIST is empty
 * 
 ****************************************************************************/
static __inline LIST_ITERATOR 
ListTail( 
	IN const DLIST* const pList )
{
	ASSERT( pList );
	return( QListTail( &pList->m_List ) );
}


/****************************************************************************
 * ListPrev
 * 
 * Description:
 *	Return a LIST_ITERATOR to the item before of the specified element.
 *	Returns NULL if there is no element before of the specified element.
 * 
 *	The LIST_ITERATOR is opaque to the user.  To get the user object
 *	at this element, use ListObj().
 * 
 * Inputs:
 *	pList		- Pointer to list object
 *	ListItem	- LIST_ITERATOR to item in the list
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	LIST_ITERATOR to item before the specified item
 * 
 ****************************************************************************/
static __inline LIST_ITERATOR 
ListPrev( 
	IN const DLIST* const pList,
	IN LIST_ITERATOR const Iterator )
{
	/* sanity checks */
	ASSERT( pList );
	ASSERT( Iterator );
	return( QListPrev( &pList->m_List, (LIST_ITEM*)Iterator ) );
}


/****************************************************************************
 * ListNext
 * 
 * Description:
 *	Return a LIST_ITERATOR to the item after the specified element.
 *	Returns NULL if there is no element after of the specified element.
 * 
 *	The LIST_ITERATOR is opaque to the user.  To get the user object
 *	at this element, use ListObj().
 * 
 * Inputs:
 *	pList		- Pointer to DLIST object
 *	ListItem	- LIST_ITERATOR to item in the list
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	LIST_ITERATOR to item before the specified item
 * 
 ****************************************************************************/
static __inline LIST_ITERATOR 
ListNext(
	IN const DLIST* const pList,
	IN LIST_ITERATOR const Iterator )
{
	/* sanity checks */
	ASSERT( pList );
	ASSERT( Iterator );
	return( QListNext( &pList->m_List, (LIST_ITEM*)Iterator ) );
}


/****************************************************************************
 * ListRemoveHead
 * 
 * Description:
 *	Remove the item at the head of the list and return the pointer to
 *	its user object.
 * 
 * Inputs:
 *	pList - Pointer to DLIST object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to the user object at the head of the list
 *	NULL if the list is empty
 * 
 ****************************************************************************/
static __inline void* 
ListRemoveHead( 
	IN DLIST* const pList )
{
	LIST_ITEM	*pListItem;

	ASSERT( pList );
	/* See if the list is empty. */
	/* Get the item at the head of the list. */
	if( (pListItem = QListRemoveHead( &pList->m_List )) == NULL )
		return( NULL );

	/* Place the LIST_ITEM back into the free list. */
	QListInsertHead( &pList->m_FreeItemList, pListItem );

	return pListItem->pObject;
}


/****************************************************************************
 * ListRemoveTail
 * 
 * Description:
 *	Remove the item at the tail of the list and return the pointer to
 *	its user object.
 * 
 * Inputs:
 *	pList - Pointer to DLIST object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to the user object at the tail of the list
 *	NULL if the list is empty
 * 
 ****************************************************************************/
static __inline void* 
ListRemoveTail( 
	IN DLIST* const pList )
{
	LIST_ITEM	*pListItem;

	ASSERT( pList );
	/* See if the list is empty. */
	/* Get the item at the tail of the list. */
	if( (pListItem = QListRemoveTail( &pList->m_List )) == NULL )
		return( NULL );

	/* Place the LIST_ITEM back into the free list. */
	QListInsertHead( &pList->m_FreeItemList, pListItem );

	return pListItem->pObject;
}


/****************************************************************************
 * ListRemoveItem
 * 
 * Description:
 *	Removes the item indicated by the LIST_ITERATOR.
 * 
 * Inputs:
 *	pList		- Pointer to list object
 *	Iterator	- LIST_ITERATOR to the list item to remove.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static __inline void 
ListRemoveItem( 
	IN DLIST* const pList,
	IN LIST_ITERATOR const Iterator )
{
	ASSERT( pList );

	QListRemoveItem( &pList->m_List, (LIST_ITEM*)Iterator );

	/* Place the LIST_ITEM back into the free list. */
	QListInsertHead( &pList->m_FreeItemList, (LIST_ITEM*)Iterator );
}



/****************************************************************************
 * ListRemoveObject
 * 
 * Description:
 *	Remove the item containing the specified user object from the DLIST.
 * 
 * Inputs:
 *	pList	- Pointer to DLIST object
 *	pObject	- User object pointer (may be NULL)
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	TRUE if the object was found and removed from the list
 *	FALSE otherwise
 * 
 ****************************************************************************/
boolean	
ListRemoveObject( 
	IN DLIST* const pList, 
	IN void* const pObject );

  
/****************************************************************************
 * ListRemoveAll
 * 
 * Description:
 *	Remove all the items in a list.
 * 
 * Inputs:
 *	pList - Pointer to DLIST object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static __inline void 
ListRemoveAll( 
	IN DLIST* const pList )
{
	ASSERT( pList );

	/* Move the whole list of items to the front of the free item list. */
	QListInsertListHead( &pList->m_FreeItemList, &pList->m_List );
}


/****************************************************************************
 * ListInsertArrayHead
 * 
 * Description:
 *	Insert an array of new items at the head of the DLIST.
 * 
 * Inputs:
 *	pList		- Pointer to DLIST object
 *	pArray		- Pointer to array
 *	ItemCount	- Number of items in the array to insert
 *	ItemSize	- Size (in bytes) of each item in the array
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	TRUE on success
 *	FALSE otherwise
 * 
 ****************************************************************************/
boolean	
ListInsertArrayHead( 
	IN DLIST* const pList, 
	IN void* const pArray, 
	IN const uint32 ItemCount, 
	IN const uint32 ItemSize );


/****************************************************************************
 * ListInsertArrayTail
 * 
 * Description:
 *	Insert an array of new items at the tail of the DLIST.
 * 
 * Inputs:
 *	pList		- Pointer to DLIST object
 *	pArray		- Pointer to array
 *	ItemCount	- number of items in the array to insert
 *	ItemSize	- Size (in bytes) of each item in the array
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	TRUE on success
 *	FALSE otherwise
 * 
 ****************************************************************************/
boolean	
ListInsertArrayTail( 
	IN DLIST* const pList, 
	IN void* const pArray, 
	IN const uint32 ItemCount, 
	IN const uint32 ItemSize);


/****************************************************************************
 * ListIsObjectInList
 * 
 * Description:
 *	Returns true if the specified user object is in the DLIST.
 * 
 * Inputs:
 *	pList	- Pointer to DLIST object
 *	pObject - User object pointer
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	TRUE if the object was found
 *	FALSE otherwise
 * 
 ****************************************************************************/
static __inline boolean 
ListIsObjectInList( 
	IN const DLIST* const pList, 
	IN void* const pObject )
{
	ASSERT( pList );
	return ((boolean)
		(QListFindFromHead( &pList->m_List, NULL, pObject ) != NULL));
}

/* Get an item at a specific location in a list. */
/**** OBSOLETE - DO NOT USE FOR NEW CODE!!! */
void* 
ListGetObjectAt( 
	IN const DLIST* const pList,
	IN const uint32 Index );

/**** Define for backward compatibility. */
/**** OBSOLETE - DO NOT USE FOR NEW CODE!!! */
#define ListFindObject	ListFindFromHead

/****************************************************************************
 * ListFindFromHead
 * 
 * Description:
 *	Use the specified function to search for an object, returns the object if 
 *	found, otherwise NULL.  The DLIST is not modified.  Execution stops if Func
 *	returns TRUE.
 * 
 * Inputs:
 *	pList	- Pointer to DLIST object
 *	pfnFunc	- Pointer to user's compare function
 *	Context	- User context for compare function (may be NULL)
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to user object for which Func returned TRUE
 *	NULL otherwise
 * 
 ****************************************************************************/
void* 
ListFindFromHead( 
	IN const DLIST* const pList,
	IN LIST_FIND_FUNC pfnFunc,
	IN void* const Context );


/****************************************************************************
 * ListFindFromTail
 * 
 * Description:
 *	Use the specified function to search for an object starting at the tail of
 *	a list.  Returns the item if found, otherwise NULL.  The list is not 
 *	modified.
 * 
 * Inputs:
 *	pList	- Pointer to DLIST object
 *	pfnFunc	- Pointer to user's compare function
 *	Context	- User context for compare function (may be NULL)
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	Pointer to user object for which Func returned TRUE
 *	NULL otherwise
 * 
 ****************************************************************************/
void* 
ListFindFromTail( 
	IN const DLIST* const pList,
	IN LIST_FIND_FUNC pfnFunc,
	IN void* const Context );


/****************************************************************************
 * ListApplyFunc
 * 
 * Description:
 *	Calls the given function for all tiems in the list.  The list is not
 *	modified.
 * 
 * Inputs:
 *	pList	- Pointer to DLIST object
 *	pfnFunc	- Pointer to user's compare function
 *	Context - User context for compare function (may be NULL)
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
void 
ListApplyFunc( 
	IN const DLIST* const pList,
	IN LIST_APPLY_FUNC pfnFunc,
	IN void* const Context );


/****************************************************************************
 ****************************************************************************
 ***********													   **********
 ***********					DECLARATION OF QUEUE			   **********
 ***********													   **********
 ****************************************************************************
 ****************************************************************************/



/*
 * Initialize a queue protected by spinlocks.
 */
static __inline void 
QueueInitState( 
	IN QUEUE* const pQueue )
{
	ListInitState( pQueue );
}


static __inline boolean 
QueueInit(
	IN QUEUE* const pQueue, 
	IN const uint32 MinSize )
{
	ASSERT( pQueue );
	return ListInit( pQueue, MinSize );
}


/*
 * Destroy a queue and all memory associated with it.
 */
static __inline void 
QueueDestroy( IN QUEUE* const pQueue )
{
	ASSERT( pQueue );
	ListDestroy( pQueue );
}


/*
 * Add a new item to a queue.
 */
static __inline boolean 
QueueInsert( IN QUEUE* const pQueue, IN void* pObject )
{
	ASSERT( pQueue );
	return ListInsertTail( pQueue, pObject );
}


/*
 * Inserts an array of items to a queue.
 */
static __inline boolean 
QueueInsertArray( 
	IN QUEUE* const pQueue, 
	IN void* const pArray, 
	IN const uint32 ItemCount, 
	IN const uint32 ItemSize )
{
	ASSERT( pQueue );
	return ListInsertArrayTail( pQueue, pArray, ItemCount, ItemSize );
}


/*
 * Remove an item from a queue.
 */
static __inline void* 
QueueRemove(
	IN QUEUE* const pQueue )
{
	ASSERT( pQueue );
	return ListRemoveHead( pQueue );
}


/*
 * Remove the specified object from a queue.
 */
static __inline boolean 
QueueRemoveObject(
	IN QUEUE* const pQueue, 
	IN void* const pObject )
{
	ASSERT( pQueue );
	return ListRemoveObject( pQueue, pObject );
}


/*
 * Removes all items in the specified queue.
 */
static __inline void 
QueueRemoveAll( 
	IN QUEUE* const pQueue )
{
	ASSERT( pQueue );
	ListRemoveAll( pQueue );
}


/*
 * Returns whether the queue is empty.
 */
static __inline uint32 
QueueCount( 
	IN const QUEUE* const pQueue )
{
	ASSERT( pQueue );
	return ListCount( pQueue );
}


/*
 * Returns the head object without removing it.
 */
static __inline void* 
QueueGetHead( 
	IN const QUEUE* const pQueue )
{
	void	*Itor;

	ASSERT( pQueue );
	Itor = ListHead( pQueue );
	if( Itor )
		return ListObj( Itor );
	else
		return NULL;
}


/*
 * Calls the given function for all items in the queue.  The items are not
 * removed from the queue.
 */
static __inline void 
QueueApplyFunc( 
	IN const QUEUE* const pQueue, 
	IN LIST_APPLY_FUNC Func, 
	IN void* const Context )
{
	ASSERT( pQueue );
	ListApplyFunc( pQueue, Func, Context );
}


/****************************************************************************
 ****************************************************************************
 ***********													   **********
 ***********					DECLARATION OF STACK			   **********
 ***********													   **********
 ****************************************************************************
 ****************************************************************************/


/*
 * Initialize a stack protected by spinlocks.
 */
static __inline void 
StackInitState( 
	IN STACK* const pStack )
{
	ListInitState( pStack );
}

static __inline boolean 
StackInit( 
	IN STACK* const pStack, 
	IN const uint32 MinSize )
{
	ASSERT( pStack );
	return ListInit( pStack, MinSize );
}


/*
 * Destroy a stack and all memory associated with it.
 */
static __inline void 
StackDestroy(
	IN STACK* const pStack )
{
	ASSERT( pStack );
	ListDestroy( pStack );
}


/*
 * Push an object onto the stack.
 */
static __inline boolean 
StackPush( 
	IN STACK* const pStack, 
	IN void* const pObject )
{
	ASSERT( pStack );
	return ListInsertHead( pStack, pObject );
}


/*
 * Push an array of objects onto a stack.
 */
static __inline boolean 
StackPushArray( 
	IN STACK* const pStack, 
	IN void* const pArray, 
	IN const uint32 ItemCount, 
	IN const uint32 ItemSize )
{
	ASSERT( pStack );
	return ListInsertArrayHead( pStack, pArray, ItemCount, ItemSize );
}


/*
 * Pop an object from a stack.
 */
static __inline void* 
StackPop( 
	IN STACK* const pStack )
{
	ASSERT( pStack );
	return ListRemoveHead( pStack );
}


/*
 * Pops all objects from the specified stack.
 */
static __inline void 
StackPopAll(
	IN STACK* const pStack )
{
	ASSERT( pStack );
	ListRemoveAll( pStack );
}


/*
 * Returns the number of items on a stack.
 */
static __inline uint32 
StackCount( 
	IN const STACK* const pStack )
{
	ASSERT( pStack );
	return ListCount( pStack );
}


/*
 * Calls the given function for all items in the stack.  The items are not
 * removed from the stack.
 */
static __inline void 
StackApplyFunc( 
	IN const STACK* const pStack, 
	IN LIST_APPLY_FUNC Func, 
	IN void* const Context )
{
	ASSERT( pStack );
	ListApplyFunc( pStack, Func, Context );
}


#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_ILIST_H_ */
