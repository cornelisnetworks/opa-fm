/* BEGIN_ICS_COPYRIGHT4 ****************************************

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

** END_ICS_COPYRIGHT4   ****************************************/
/* [ICS VERSION STRING: unknown] */

/* declarations shared by kernel and user mode CMs */

/* Suppress duplicate loading of this file */
#ifndef _IBA_IB_CM_COMMON_H_
#define _IBA_IB_CM_COMMON_H_


#if defined (__cplusplus)
extern "C" {
#endif

/* OS independent routines */
#include "datatypes.h"
#include "ievent.h"
#include "itimer.h"
#include "imemory.h"
#include "ispinlock.h"
#include "idebug.h"
#include "ithread.h"
#include "isyscallback.h"
#include "iquickmap.h"
#include "imath.h"
#include "ib_cm.h"
#include "cm_params.h"


/*************************************************************************
 * Defines
 */

/* The following debug macros must be defined before including the "ib_debug.h" */
#define	_DBG_PRINT_EPILOG(LEVEL,STRING)
#define	_DBG_PRINT_PROLOG(LEVEL,STRING)
#define	_DBG_ERROR_EPILOG(LEVEL,STRING)
#define	_DBG_ERROR_PROLOG(LEVEL,STRING)
#define	_DBG_FATAL_EPILOG(LEVEL,STRING)
#define	_DBG_FATAL_PROLOG(LEVEL,STRING)
#define _DBG_CHK_IRQL(IRQL)

#include "ib_debug.h"

#undef _ib_dbg_params
#define _ib_dbg_params cm_debug_params
extern IB_DBG_PARAMS	cm_debug_params;

/* Memory tag to use in all memory allocation */
#define CM_MEM_TAG						0xbeef /*_ib_dbg_params.mem_tag */

/* Generic macros to test, set or clear bits */
#define BitTest(a,b)					((a) & (b))
#define BitSet(a,b)						((a) |= (b))
#define BitClear(a,b)					((a) &= (~(b)))

#define UNUSED_VAR(x)	(void)(x)

// forward type decl
struct _IBT_COMPONENT_INFO; 

/*************************************************************************
 * Doubly link-list macros
 */
typedef struct DLIST_ENTRY {
   struct DLIST_ENTRY *Next;
   struct DLIST_ENTRY *Prev;
} DLIST_ENTRY, *PDLIST_ENTRY;

/*
 *  void
 *  DListInit(
 *      PDLIST_ENTRY ListHead
 *      );
 */
#define DLIST_INIT	DListInit

#define DListInit(ListHead) (\
    (ListHead)->Next = (ListHead)->Prev = (ListHead))


/*
 *  boolean
 *  DListIsEmpty(
 *      PDLIST_ENTRY ListHead
 *      );
 */
#define DLIST_IS_EMPTY			DListIsEmpty

#define DListIsEmpty(ListHead) \
    ((ListHead)->Next == (ListHead))


/*
 *  PDLIST_ENTRY
 *  DListGetNext(
 *      PDLIST_ENTRY Entry
 *      );
 */
#define	DLIST_GET_NEXT 			DListGetNext

#define DListGetNext(Entry) \
    (Entry)->Next


/*
 *  PDLIST_ENTRY
 *  DListGetPrev(
 *      PDLIST_ENTRY Entry
 *      );
 */
#define	DLIST_GET_PREV			DListGetPrev

#define DListGetPrev(Entry) \
    (Entry)->Prev


/*
 *  PDLIST_ENTRY
 *  DListGetHead(
 *      PDLIST_ENTRY ListHead
 *      );
 */
#define	DLIST_GET_HEAD			DListGetHead

#define DListGetHead(ListHead) \
    (ListHead)->Next



/*
 *  PDLIST_ENTRY
 *  DListRemoveHead(
 *      PDLIST_ENTRY ListHead
 *      );
 */

#define	DLIST_REMOVE_HEAD		DListRemoveHead

#define DListRemoveHead(ListHead, List, Obj) \
    (ListHead)->Next;\
    {DListRemoveEntry((ListHead)->Next, List, Obj)}


/*
 *  PDLIST_ENTRY
 *  DListRemoveTail(
 *      PDLIST_ENTRY ListHead
 *      );
 */
#define	DLIST_REMOVE_TAIL		DListRemoveTail

#define DListRemoveTail(ListHead, List, Obj) \
    (ListHead)->Prev;\
    {DListRemoveEntry((ListHead)->Prev, List, Obj)}


/*
 *  void
 *  DListRemoveEntry(
 *      PDLIST_ENTRY Entry
 *      );
 */
#define	DLIST_REMOVE_ENTRY		DListRemoveEntry

#define DListRemoveEntry(Entry, List, Obj) {\
    PDLIST_ENTRY pNext = (Entry)->Next;\
    PDLIST_ENTRY pPrev = (Entry)->Prev;\
    pPrev->Next = pNext;\
    pNext->Prev = pPrev;\
	DListInit(Entry);\
	_DBG_INFO(("<cep 0x%p> --> Removed from " #List "\n", _DBG_PTR(Obj)));\
	}

/*
 *  void
 *  DListInsertTail(
 *      PDLIST_ENTRY ListHead,
 *      PDLIST_ENTRY Entry
 *      );
 */

#define	DLIST_INSERT_TAIL		DListInsertTail

#define DListInsertTail(ListHead, Entry, List, Obj) {\
    PDLIST_ENTRY pHead = (ListHead);\
    PDLIST_ENTRY pPrev = (ListHead)->Prev;\
    (Entry)->Next = pHead;\
    (Entry)->Prev = pPrev;\
    pPrev->Next = (Entry);\
    pHead->Prev = (Entry);\
	_DBG_INFO(("<cep 0x%p> --> Insert into " #List "\n", _DBG_PTR(Obj)));\
    }


/*
 *  void
 *  DListInsertHead(
 *      PDLIST_ENTRY ListHead,
 *      PDLIST_ENTRY Entry
 *      );
 */
#define	DLIST_INSERT_HEAD		DListInsertHead

#define DListInsertHead(ListHead, Entry, List, Obj) {\
    PDLIST_ENTRY pHead = (ListHead);\
    PDLIST_ENTRY pNext = (ListHead)->Next;\
    (Entry)->Next = pNext;\
    (Entry)->Prev = pHead;\
    pNext->Prev = (Entry);\
    pHead->Next = (Entry);\
	_DBG_INFO(("<cep 0x%p> --> Insert into " #List "\n", _DBG_PTR(Obj)));\
    }


/*
 *  void
 *  DListIterate(
 *      PDLIST_ENTRY anchor,
 *      PDLIST_ENTRY index,
 *		PDLIST_ENTRY listp
 *      );
 */

#define	DLIST_ITERATE	DListIterate

#define DListIterate(anchor, index, listp) \
	(anchor) = (PDLIST_ENTRY)(listp); \
	for((index) = (anchor)->Next; (index) != (anchor); (index) = (index)->Next)

/*************************************************************************
 * Keyed Map macros
 */

#define CM_MAP cl_qmap_t
#define CM_MAP_ITEM cl_map_item_t

#define CM_MapInit(pMap, pFunc)	cl_qmap_init(pMap, pFunc)
#define CM_MapHead(pMap)		cl_qmap_head(pMap)
#define CM_MapEnd(pMap)			cl_qmap_end(pMap)
#define CM_MapNext(pMapEntry)	cl_qmap_next(pMapEntry)
#define CM_MapPrev(pMapEntry)	cl_qmap_prev(pMapEntry)
#define CM_MapGet(pMap, Key)	cl_qmap_get(pMap, Key)
#define CM_MapGetCompare(pMap, Key, pFunc) cl_qmap_get_compare(pMap, Key, pFunc)
#define CM_MapIsEmpty(pMap)		cl_is_qmap_empty(pMap)

#define CM_MapInsert(pMap, Key, pEntry, MapName, pCEP) {\
	CM_MAP_ITEM *pMapEntry; \
	pMapEntry = cl_qmap_insert(pMap, Key, pEntry); \
	/* If duplicate key, insert does nothing and returns existing entry */ \
	ASSERT(pMapEntry == pEntry); \
	_DBG_INFO(("<cep 0x%p> --> Insert into " #MapName "\n", _DBG_PTR(pCEP)));\
	}

#define CM_MapRemoveEntry(pMap, pEntry, MapName, pCEP) {\
	cl_qmap_remove_item(pMap, pEntry); \
	_DBG_INFO(("<cep 0x%p> --> Removed from " #MapName "\n", _DBG_PTR(pCEP)));\
	}


/*************************************************************************/

/*************************************************************************
 * Function prototypes
 */


/*
 * CmInit
 *
 *	Initialize the CM module. This should only be called once normally during
 *	system startup time.
 *
 *
 *
 *
 * INPUTS:
 *
 *	None.
 *	
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *	FERROR
 *	FINSUFFICIENT_RESOURCES
 *
 */
FSTATUS
CmInit(
	IN struct _IBT_COMPONENT_INFO *ComponentInfo
	);


/*
 * CmDestroy
 *
 *	Destroy the CM module. This should only be called once normally during
 *	system shutdown time to cleanup any resources used by the CM.
 *
 *
 *
 *
 * INPUTS:
 *
 *	None.
 *	
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *	FERROR
 *
 */
void
CmDestroy( void );


/*
 * CmGetConnInfo
 *
 *	This routine is used to retrieve the connection status and information
 *	on the specified endpoint.  This is intended for Internal use only as part
 *  of generating the arguments for CM callbacks
 *
 *  Internal use only
 *
 * INPUTS:
 *
 *	hCEP				- Handle to the CEP object.
 *
 *	pConnInfo			- Specify the buffer to store the connection information and status.
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *
 *	FERROR
 *
 */
FSTATUS
CmGetConnInfo(
	IN IB_HANDLE				hCEP,
	OUT CM_CONN_INFO*			pConnInfo
	);


#if defined (__cplusplus)
};
#endif


#endif  /* _IBA_IB_CM_COMMON_H_ */
