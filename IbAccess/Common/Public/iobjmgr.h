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

#ifndef _IBA_PUBLIC_IOBJMGR_H_
#define _IBA_PUBLIC_IOBJMGR_H_

#include "iba/public/ilist.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * An object manager allocates objects, then deallocates them when destroyed.
 */
typedef struct _OBJECT_MGR
{
	/* Private data structure for the object manager. */
	/* Users should not access these variables directly. */
	QUICK_LIST	m_ObjList;			/* DO NOT USE!! */
	boolean		ObjectsPageable;	/* DO NOT USE!! */
	boolean		m_Initialized;		/* DO NOT USE!! */

} OBJECT_MGR;


/* Create an object manager. */
void 
ObjMgrInitState( 
	IN OBJECT_MGR* const pObjMgr );

boolean 
ObjMgrInit( 
	IN OBJECT_MGR* const pObjMgr, 
	IN const boolean ObjectsPageable );

/* Destroy an object manager and all allocated resources. */
void 
ObjMgrDestroy( 
	IN OBJECT_MGR* const pObjMgr );

/* Allocate an object and add it to the object manager's list. */
void* 
ObjMgrAllocate( 
	IN OBJECT_MGR* const pObjMgr, 
	IN const uint32 Bytes );

/* Destroy a specific object maintained by the object manager. */
void 
ObjMgrDeallocate( 
	IN OBJECT_MGR* const pObjMgr, 
	IN void* const pObject );


#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_IOBJMGR_H_ */
