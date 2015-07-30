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

#include "iobjmgr.h"
#include "imemory.h"


#define OBJ_MGR_TAG		MAKE_MEM_TAG( i, o, b, j )

void 
ObjMgrInitState(
	IN OBJECT_MGR* const pObjMgr )
{
	QListInitState( &pObjMgr->m_ObjList );
	pObjMgr->m_Initialized = FALSE;
}


boolean 
ObjMgrInit( 
	IN OBJECT_MGR* const pObjMgr, 
	IN const boolean ObjectsPageable )
{
	ASSERT( pObjMgr );

	pObjMgr->m_Initialized = FALSE;
	pObjMgr->ObjectsPageable = ObjectsPageable;
	pObjMgr->m_Initialized = 
		QListInit( &pObjMgr->m_ObjList );
	return pObjMgr->m_Initialized;
}


void 
ObjMgrDestroy( 
	IN OBJECT_MGR* const pObjMgr )
{
	LIST_ITEM	*pListItem;
	ASSERT( pObjMgr );

	if( !pObjMgr->m_Initialized )
		return;

	// Deallocate all objects.
	while( NULL != (pListItem = QListRemoveHead( &pObjMgr->m_ObjList )) )
		MemoryDeallocate( pListItem );

	QListDestroy( &pObjMgr->m_ObjList );
	pObjMgr->m_Initialized = FALSE;
}


// Allocate an object and add it to the object manager's list.
void* 
ObjMgrAllocate( 
	IN OBJECT_MGR* const pObjMgr, 
	IN const uint32 Bytes )
{
	LIST_ITEM	*pListItem;
	ASSERT( pObjMgr && Bytes && pObjMgr->m_Initialized );

	// Allocate the object, with a list item at the head.
	pListItem = (LIST_ITEM*)MemoryAllocateAndClear( Bytes + sizeof( LIST_ITEM ),
		pObjMgr->ObjectsPageable, OBJ_MGR_TAG );

	if( !pListItem )
		return NULL;

	// Insert the item into the object manager's list.
	pListItem->pObject = (char*)pListItem + sizeof( LIST_ITEM );
	QListInsertHead( &pObjMgr->m_ObjList, pListItem );
	return pListItem->pObject;
}


// Destroy an object maintained by the object manager.
void 
ObjMgrDeallocate(
	IN OBJECT_MGR* const pObjMgr, 
	IN void* const pObject )
{
	LIST_ITEM	*pListItem;
	ASSERT( pObjMgr && pObject && pObjMgr->m_Initialized );

	// Remove the object from the list.
	pListItem = (LIST_ITEM*)((char*)pObject - sizeof( LIST_ITEM ));
	ASSERT( pListItem->pObject == pObject );
	QListRemoveItem( &pObjMgr->m_ObjList, pListItem );

	// Deallocate the object.
	MemoryDeallocate( pListItem );
}
